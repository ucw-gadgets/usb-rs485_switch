/*
 *	USB-RS485 Switch -- MODBUS Channels
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "firmware.h"

struct channel {
	struct message_queue send_queue;
	struct message_node *current;
};

static struct channel channels[2];

enum mb_error {
	ERR_ILLEGAL_FUNCTION = 0x01,
	ERR_ILLEGAL_DATA_ADDRESS = 0x02,
	ERR_ILLEGAL_DATA_VALUE = 0x03,
	ERR_SLAVE_DEVICE_FAILURE = 0x04,
	ERR_GATEWAY_PATH_UNAVAILABLE = 0x0a,
	ERR_GATEWAY_TARGET_NO_RESPONSE = 0x0b,
};

bool set_port_params(uint port, struct urs485_port_params *par)
{
	if (par->baud_rate < 1200 || par->baud_rate > 115200 ||
	    par->parity > 2 ||
	    par->powered > 1 ||
	    !par->request_timeout)
		return false;

	port_params[port] = *par;

	if (par->powered)
		reg_set_flag(port, SF_PWREN);
	else
		reg_clear_flag(port, SF_PWREN);
	reg_send();

	return true;
}

static void internal_error_reply(struct message_node *n, byte error_code)
{
	struct urs485_message *m = &n->msg;
	debug_printf("Msg #%04x: Internal error %d\n", m->message_id, error_code);
	m->frame_size = 2;
	m->frame[0] |= 0x80;
	m->frame[1] = error_code;
	queue_put(&done_queue, n);
}

void got_msg_from_usb(struct message_node *m)
{
	struct urs485_message *g = &m->msg;

	if (g->port < 8 && g->frame_size >= 2 && g->frame_size <= 2 + MODBUS_MAX_DATA_SIZE)
		queue_put(&channels[g->port / 4].send_queue, m);
	else
		internal_error_reply(m, ERR_GATEWAY_PATH_UNAVAILABLE);
}
