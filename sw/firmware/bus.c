/*
 *	USB-RS485 Switch -- MODBUS Channels
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "firmware.h"
#include "modbus-proto.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include <string.h>

// Debugging
#define MODBUS_DEBUG

#ifdef MODBUS_DEBUG
#define DEBUG debug_printf
#define CDEBUG(c, msg, ...) debug_printf("MODBUS%d: " msg, c->id, ## __VA_ARGS__)
#else
#define DEBUG( msg, ...) do { } while (0)
#define CDEBUG(c, msg...) do { } while (0)
#endif

struct channel {
	struct message_queue send_queue;
	struct message_node *current;
	u32 usart;
	u32 timer;

	byte id;
	byte state;			// STATE_xxx
	byte active_port;		// 0xff if none
	bool port_stale;		// active port needs reconfiguration

	struct port_state *port_state;	// of active port
	struct urs485_port_status *port_status;

	u16 rx_char_timeout;
	u16 inter_frame_gap;

	u32 transaction_start_time;	// time at the start of transaction
	u32 transaction_end_time;	// ... at the end of transaction

	byte *tx_buf;
	u16 tx_pos;
	u16 tx_size;

	byte rx_buf[256];
	u16 rx_size;
	u16 rx_timeout;			// in ms
	u32 rx_start_at;		// ms_ticks when RX started
	byte rx_bad;
};

static struct channel channels[2];

enum mb_state {
	STATE_IDLE,			// iff channel->current == NULL
	STATE_GAP,			// inter-frame gap
	STATE_TX,
	STATE_TX_LAST,
	STATE_TX_DONE,
	STATE_RX,
	STATE_RX_DONE,
	STATE_BROADCAST_DONE,
};

enum mb_rx_bad {
	RX_BAD_OK,
	RX_BAD_CHAR,			// parity error, serial port overrun etc.
	RX_BAD_OVERSIZE,
};

// State related to port LEDs
static byte led_active_mask;		// ports currently active
static byte led_history_mask;		// ports active since last recalculation

bool set_port_params(uint port, struct urs485_port_params *par)
{
	if (par->baud_rate < 1200 || par->baud_rate > 115200 ||
	    par->parity > 2 ||
	    par->powered > 1 ||
	    !par->request_timeout)
		return false;

	DEBUG("Setting up port %u (rate=%u, par=%u, power=%u, timeout=%u)\n",
		port, (uint) par->baud_rate, par->parity, par->powered, par->request_timeout);
	ports[port].params = *par;

	if (par->powered)
		reg_set_flag(port, SF_PWREN);
	else
		reg_clear_flag(port, SF_PWREN);
	reg_send();

	struct channel *c = &channels[port/4];
	if (c->active_port == port)
		c->port_stale = true;

	return true;
}

static void internal_error_reply(struct message_node *n, byte error_code)
{
	struct urs485_message *m = &n->msg;
	DEBUG("Msg #%04x: Internal error %d\n", m->message_id, error_code);
	m->frame_size = 3;
	m->frame[1] |= 0x80;
	m->frame[2] = error_code;
	queue_put(&done_queue, n);
}

static void channel_activate_port(struct channel *c, uint port)
{
	if (c->active_port != port || c->port_stale) {
		CDEBUG(c, "Activating port %d\n", port);

		if (c->active_port != 0xff)
			reg_set_flag(c->active_port, SF_RXEN_N);

		struct port_state *state = &ports[port];
		struct urs485_port_params *par = &state->params;
		usart_disable(c->usart);
		usart_set_baudrate(c->usart, par->baud_rate);
		switch (par->parity) {
			default:	// none
				usart_set_databits(c->usart, 8);
				usart_set_stopbits(c->usart, USART_STOPBITS_2);
				usart_set_parity(c->usart, USART_PARITY_NONE);
				break;
			case 1:		// odd
				usart_set_databits(c->usart, 9);
				usart_set_stopbits(c->usart, USART_STOPBITS_1);
				usart_set_parity(c->usart, USART_PARITY_ODD);
				break;
			case 2:		// even
				usart_set_databits(c->usart, 9);
				usart_set_stopbits(c->usart, USART_STOPBITS_1);
				usart_set_parity(c->usart, USART_PARITY_EVEN);
				break;
		}
		usart_set_flow_control(c->usart, USART_FLOWCONTROL_NONE);
		usart_enable(c->usart);

		if (par->baud_rate <= 19200) {
			// For low baud rates, the standard specifies timeout of 1.5 character times
			// (1 character = start bit + 8 data bits + parity bit + stop bit = 11 bits).
			// Minimum inter-frame delay is 3.5 character times.
			c->rx_char_timeout = 1000000*11*3/2/par->baud_rate;
			c->inter_frame_gap = 1000000*11*7/2/par->baud_rate;
		} else {
			// For high rates, the timeout is fixed to 750 μs and inter-frame delay to 1750 μs.
			c->rx_char_timeout = 750;
			c->inter_frame_gap = 1750;
		}

		c->active_port = port;
		c->port_stale = false;
		c->rx_timeout = par->request_timeout;
		c->port_state = state;
		c->port_status = &state->status;
	}

	led_active_mask |= 1U << port;
	led_history_mask |= 1U << port;
}

static void channel_deactivate_port(struct channel *c)
{
	// As an optimization, we do not really change hardware state here.
	// Everything is handled at the next channel activation.

	led_active_mask &= ~(1U << c->active_port);
}

static void channel_tx_init(struct channel *c)
{
	c->state = STATE_TX;
	c->transaction_start_time = get_current_time();
	c->tx_buf = c->current->msg.frame;
	c->tx_pos = 0;
	c->tx_size = c->current->msg.frame_size + 2;

	usart_set_mode(c->usart, USART_MODE_TX);
	usart_enable_tx_interrupt(c->usart);

	reg_set_flag(c->active_port, SF_TXEN | SF_RXEN_N);
	reg_send();
}

static void channel_tx_done(struct channel *c)
{
	c->state = STATE_TX_DONE;
	// usart_disable_tx_interrupt(c->usart);		// Already done by irq handler
}

static void channel_tx_gap(struct channel *c)
{
	u32 gap = get_current_time() - c->port_state->last_transaction_end_time;
	u32 minimum_gap = (c->inter_frame_gap - c->rx_char_timeout) * MICROSECOND;
	if (gap + MICROSECOND >= minimum_gap) {
		channel_tx_init(c);
	} else {
		/*
		 *  It is possible that if the port was idle for a long time,
		 *  the current time has overflowed. In that case, we might
		 *  insert an unnecessary delay. Since the delay is short and
		 *  the probablity of this happening quite small, we do not care.
		 */
		CDEBUG(c, "Inter-frame gap: %u ticks\n", (uint)(minimum_gap - gap));
		c->state = STATE_GAP;
		timer_set_period(c->timer, (minimum_gap - gap) / MICROSECOND);	// at least 1
		timer_generate_event(c->timer, TIM_EGR_UG);
		timer_enable_counter(c->timer);
	}
}

static void channel_rx_init(struct channel *c)
{
	c->state = STATE_RX;
	c->rx_size = 0;
	c->rx_bad = RX_BAD_OK;
	c->rx_start_at = ms_ticks;

	reg_clear_flag(c->active_port, SF_TXEN | SF_RXEN_N);
	reg_send();

	usart_set_mode(c->usart, USART_MODE_RX);
	usart_enable_rx_interrupt(c->usart);
}

static void channel_rx_done(struct channel *c)
{
	c->state = STATE_RX_DONE;
	usart_disable_rx_interrupt(c->usart);
	usart_set_mode(c->usart, 0);
	timer_disable_counter(c->usart);
	c->transaction_end_time = get_current_time();
}

static void channel_timer_isr(struct channel *c)
{
	if (TIM_SR(c->timer) & TIM_SR_UIF) {
		TIM_SR(c->timer) &= ~TIM_SR_UIF;
		if (c->state == STATE_RX)
			channel_rx_done(c);
		else if (c->state == STATE_GAP)
			channel_tx_init(c);
	}
}

static void channel_usart_isr(struct channel *c)
{
	u32 status = USART_SR(c->usart);

	if (status & USART_SR_RXNE) {
		uint ch = usart_recv(c->usart);
		if (c->state == STATE_RX) {
			if (status & (USART_SR_FE | USART_SR_ORE | USART_SR_NE)) {
				c->rx_bad = RX_BAD_CHAR;
			} else if (c->rx_size < 256) {
				c->rx_buf[c->rx_size++] = ch;
			} else {
				// Frame too long
				c->rx_bad = RX_BAD_OVERSIZE;
			}
			timer_set_period(c->timer, c->rx_char_timeout);
			timer_generate_event(c->timer, TIM_EGR_UG);
			timer_enable_counter(c->timer);
		}
	}

	if (c->state == STATE_TX) {
		if (status & USART_SR_TXE) {
			if (c->tx_pos < c->tx_size) {
				usart_send(c->usart, c->tx_buf[c->tx_pos++]);
			} else {
				// The transmitter is double-buffered, so at this moment, it is transmitting
				// the last byte of the frame. Wait until transfer is completed.
				usart_disable_tx_interrupt(c->usart);
				USART_CR1(c->usart) |= USART_CR1_TCIE;
				c->state = STATE_TX_LAST;
			}
		}
	} else if (c->state == STATE_TX_LAST) {
		if (status & USART_SR_TC) {
			// Transfer of the last byte is complete. Release the bus.
			USART_CR1(c->usart) &= ~USART_CR1_TCIE;
			channel_tx_done(c);
			if (c->tx_buf[0])
				channel_rx_init(c);
			else {
				c->state = STATE_BROADCAST_DONE;
				c->transaction_end_time = get_current_time();
			}
		}
	}
}

void tim2_isr(void)
{
	channel_timer_isr(&channels[0]);
}

void tim3_isr(void)
{
	channel_timer_isr(&channels[1]);
}

void usart1_isr(void)
{
	channel_usart_isr(&channels[0]);
}

void usart3_isr(void)
{
	channel_usart_isr(&channels[1]);
}

/*** CRC ***/

static const byte crc_hi[] = {
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40,
	0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40,
	0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40,
	0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40
};

static const byte crc_lo[] = {
	0x00, 0xc0, 0xc1, 0x01, 0xc3, 0x03, 0x02, 0xc2, 0xc6, 0x06,
	0x07, 0xc7, 0x05, 0xc5, 0xc4, 0x04, 0xcc, 0x0c, 0x0d, 0xcd,
	0x0f, 0xcf, 0xce, 0x0e, 0x0a, 0xca, 0xcb, 0x0b, 0xc9, 0x09,
	0x08, 0xc8, 0xd8, 0x18, 0x19, 0xd9, 0x1b, 0xdb, 0xda, 0x1a,
	0x1e, 0xde, 0xdf, 0x1f, 0xdd, 0x1d, 0x1c, 0xdc, 0x14, 0xd4,
	0xd5, 0x15, 0xd7, 0x17, 0x16, 0xd6, 0xd2, 0x12, 0x13, 0xd3,
	0x11, 0xd1, 0xd0, 0x10, 0xf0, 0x30, 0x31, 0xf1, 0x33, 0xf3,
	0xf2, 0x32, 0x36, 0xf6, 0xf7, 0x37, 0xf5, 0x35, 0x34, 0xf4,
	0x3c, 0xfc, 0xfd, 0x3d, 0xff, 0x3f, 0x3e, 0xfe, 0xfa, 0x3a,
	0x3b, 0xfb, 0x39, 0xf9, 0xf8, 0x38, 0x28, 0xe8, 0xe9, 0x29,
	0xeb, 0x2b, 0x2a, 0xea, 0xee, 0x2e, 0x2f, 0xef, 0x2d, 0xed,
	0xec, 0x2c, 0xe4, 0x24, 0x25, 0xe5, 0x27, 0xe7, 0xe6, 0x26,
	0x22, 0xe2, 0xe3, 0x23, 0xe1, 0x21, 0x20, 0xe0, 0xa0, 0x60,
	0x61, 0xa1, 0x63, 0xa3, 0xa2, 0x62, 0x66, 0xa6, 0xa7, 0x67,
	0xa5, 0x65, 0x64, 0xa4, 0x6c, 0xac, 0xad, 0x6d, 0xaf, 0x6f,
	0x6e, 0xae, 0xaa, 0x6a, 0x6b, 0xab, 0x69, 0xa9, 0xa8, 0x68,
	0x78, 0xb8, 0xb9, 0x79, 0xbb, 0x7b, 0x7a, 0xba, 0xbe, 0x7e,
	0x7f, 0xbf, 0x7d, 0xbd, 0xbc, 0x7c, 0xb4, 0x74, 0x75, 0xb5,
	0x77, 0xb7, 0xb6, 0x76, 0x72, 0xb2, 0xb3, 0x73, 0xb1, 0x71,
	0x70, 0xb0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9c, 0x5c,
	0x5d, 0x9d, 0x5f, 0x9f, 0x9e, 0x5e, 0x5a, 0x9a, 0x9b, 0x5b,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4b, 0x8b,
	0x8a, 0x4a, 0x4e, 0x8e, 0x8f, 0x4f, 0x8d, 0x4d, 0x4c, 0x8c,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static u16 crc16(byte *buf, u16 len)
{
	byte hi = 0xff, lo = 0xff;

	while (len--) {
		byte i = hi ^ *buf++;
		hi = lo ^ crc_hi[i];
		lo = crc_lo[i];
	}

	return (hi << 8 | lo);
}

/*** Upper layer ***/

static bool channel_check_rx(struct channel *c)
{
	if (c->rx_bad) {
		CDEBUG(c, "RX bad %d after %d\n", c->rx_bad, c->rx_size);
		if (c->rx_bad == RX_BAD_CHAR)
			c->port_status->cnt_frame_errors++;
		else
			c->port_status->cnt_oversize_errors++;
		return false;
	}
	
	if (c->rx_size < 4) {
		CDEBUG(c, "RX undersize %d\n", c->rx_size);
		c->port_status->cnt_undersize_errors++;
		return false;
	}

	u16 crc = crc16(c->rx_buf, c->rx_size - 2);
	u16 rx_crc = (c->rx_buf[c->rx_size-2] << 8) | c->rx_buf[c->rx_size-1];
	if (crc != rx_crc) {
		CDEBUG(c, "Bad CRC\n");
		c->port_status->cnt_crc_errors++;
		return false;
	}

	if (c->current->msg.frame[0] != c->rx_buf[0]) {
		CDEBUG(c, "Bad sender\n");
		c->port_status->cnt_mismatch_errors++;
		return false;
	}

	return true;
}

static void channel_rx_frame(struct channel *c)
{
	u32 time_delta = c->transaction_end_time - c->transaction_start_time;

	if (!channel_check_rx(c)) {
		internal_error_reply(c->current, MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED);
	} else {
		struct urs485_message *m = &c->current->msg;
		m->frame_size = c->rx_size - 2;
		memcpy(m->frame, c->rx_buf, m->frame_size);
		CDEBUG(c, "Msg #%04x: Received %d bytes (time=%u)\n", m->message_id, c->rx_size, (uint) time_delta);
		c->port_status->cnt_unicasts++;
	}

	queue_put(&done_queue, c->current);
	c->state = STATE_IDLE;
	c->current = NULL;
	c->port_state->last_transaction_end_time = c->transaction_end_time;
	channel_deactivate_port(c);
}

static void channel_broadcast_done(struct channel *c)
{
	struct urs485_message *m = &c->current->msg;
	m->frame_size = 2;
	m->frame[1] = 0;
	CDEBUG(c, "Msg #%04x: Broadcast done\n", m->message_id);

	queue_put(&done_queue, c->current);
	c->state = STATE_IDLE;
	c->current = NULL;
	c->port_status->cnt_broadcasts++;
	c->port_state->last_transaction_end_time = c->transaction_end_time;
	channel_deactivate_port(c);
}

static void channel_check_timeout(struct channel *c)
{
	if (ms_ticks - c->rx_start_at <= c->rx_timeout)
		return;

	if (c->rx_size) {
		// If we are receiving a packet which is unlike line noise, continue
		if (c->rx_bad != RX_BAD_OVERSIZE)
			return;
		c->port_status->cnt_oversize_errors++;
	}

	channel_rx_done(c);
	internal_error_reply(c->current, MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED);
	c->state = STATE_IDLE;
	c->current = NULL;
	c->port_status->cnt_timeouts++;
	channel_deactivate_port(c);
}

static void channel_idle(struct channel *c)
{
	if (queue_is_empty(&c->send_queue))
		return;

	c->current = queue_get(&c->send_queue);

	struct urs485_message *m = &c->current->msg;
	u16 crc = crc16(m->frame, m->frame_size);
	m->frame[m->frame_size] = crc >> 8;
	m->frame[m->frame_size + 1] = crc;
	CDEBUG(c, "Msg #%04x: Sending %d bytes to port %d\n", m->message_id, m->frame_size + 2, c->current->msg.port);

	channel_activate_port(c, c->current->msg.port);
	channel_tx_gap(c);
}

static void channel_loop(struct channel *c)
{
	switch (c->state) {
		case STATE_RX:
			channel_check_timeout(c);
			break;
		case STATE_RX_DONE:
			channel_rx_frame(c);
			break;
		case STATE_BROADCAST_DONE:
			channel_broadcast_done(c);
			break;
		default: ;
	}

	if (c->state == STATE_IDLE)
		channel_idle(c);
}

static void bus_update_leds(void)
{
	for (uint i=0; i<8; i++) {
		if (led_history_mask & (1U << i))
			reg_set_flag(i, SF_LED);
		else
			reg_clear_flag(i, SF_LED);
	}
	reg_send();

	led_history_mask = led_active_mask;
}

void bus_loop(void)
{
	static u32 last_leds = 3;

	channel_loop(&channels[0]);
	channel_loop(&channels[1]);

	if (ms_ticks - last_leds >= 100) {
		bus_update_leds();
		last_leds = ms_ticks;
	}
}

void got_msg_from_usb(struct message_node *n)
{
	struct urs485_message *m = &n->msg;

	if (m->port < 8 && m->frame_size >= 2 && m->frame_size <= 2 + MODBUS_MAX_DATA_SIZE)
		queue_put(&channels[m->port / 4].send_queue, n);
	else
		internal_error_reply(n, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);
}

static void channel_init(struct channel *c)
{
	timer_set_prescaler(c->timer, CPU_CLOCK_MHZ-1);	// 1 tick = 1 μs
	timer_set_mode(c->timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);
	timer_update_on_overflow(c->timer);
	timer_disable_preload(c->timer);
	timer_one_shot_mode(c->timer);
	timer_enable_irq(c->timer, TIM_DIER_UIE);

	c->active_port = 0xff;
	c->state = STATE_IDLE;
}

void bus_init(void)
{
	// Channel 0: USART1 (PA9 = TXD1, PA10 = RXD1)
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);
	channels[0].id = 0;
	channels[0].usart = USART1;
	channels[0].timer = TIM2;
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	channel_init(&channels[0]);

	// Channel 1: USART3 (PB10 = TXD3, PB11 = RXD3)
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);
	channels[1].id = 1;
	channels[1].usart = USART3;
	channels[1].timer = TIM3;
	nvic_enable_irq(NVIC_USART3_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	channel_init(&channels[1]);
}
