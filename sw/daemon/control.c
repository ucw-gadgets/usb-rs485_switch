/*
 *	USB-RS485 Switch Daemon -- Control Commands
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 *
 *	Based on parts of the MODBUS module from stm32lib.
 */

#define LOCAL_DEBUG

#include "daemon.h"

// States of processing a control message
enum control_state {
	CSTATE_INIT,
	CSTATE_USB_READ,	// Waiting for data provided by USB
	CSTATE_USB_WRITE,	// Waiting for data sent to USB
	CSTATE_DONE,
};

// Context of processing a control message
struct ctrl {
	struct message *msg;
	struct port *for_port;
	enum control_state state;
	byte *rpos, *rend;
	byte *wpos;
	bool need_get_port_status;
	bool need_set_port_params;
};

#define CTRL_DBG(ctrl_ctx, fmt, ...) DBG("CTRL(%s): " fmt, ctrl_ctx->for_port->box->cf->name, ##__VA_ARGS__)

static uint read_remains(struct ctrl *c)
{
	return c->rend - c->rpos;
}

static byte read_byte(struct ctrl *c)
{
	return *c->rpos++;
}

static u16 read_u16(struct ctrl *c)
{
	byte hi = *c->rpos++;
	byte lo = *c->rpos++;
	return (hi << 8) | lo;
}

static void write_byte(struct ctrl *c, uint v)
{
	*c->wpos++ = v;
}

static void write_u16(struct ctrl *c, u16 v)
{
	write_byte(c, v >> 8);
	write_byte(c, v);
}

static bool body_fits(uint body_len)
{
	// body_len excludes slave address and function code
	return body_len <= MODBUS_MAX_DATA_SIZE;
}

static void report_error(struct ctrl *c, uint code)
{
	// Discard the partially constructed body of the reply and rewrite the header
	CTRL_DBG(c, "Error %u", code);
	struct message *m = c->msg;
	m->reply[1] |= 0x80;
	m->reply[2] = code;
	c->wpos = m->reply + 3;
	c->state = CSTATE_DONE;
}

static bool check_input_register_addr(struct ctrl *c, uint addr)
{
	if (addr == 1) {
		c->need_get_port_status = true;
		return true;
	}
	return false;
}

static uint get_input_register(struct ctrl *c, uint addr)
{
	struct port *port = c->for_port;

	switch (addr) {
		case 1:
			return port->current_sense;
		default:
			ASSERT(0);
	}
}

static bool check_holding_register_addr(struct ctrl *c UNUSED, uint addr)
{
	return (addr >= 1 && addr <= 4);
}

static uint get_holding_register(struct ctrl *c, uint addr)
{
	struct port *port = c->for_port;

	switch (addr) {
		case 1:
			return port->baud_rate / 100;
		case 2:
			return port->parity;
		case 3:
			return port->powered;
		case 4:
			return port->request_timeout;
		default:
			ASSERT(0);
	}
}

static bool check_holding_register_write(struct ctrl *c UNUSED, uint addr, uint val)
{
	switch (addr) {
		case 1:
			return (val >= 12 && val <= 1152);
		case 2:
			return (val <= 2);
		case 3:
			return (val <= 1);
		case 4:
			return (val >= 1 && val <= 65535);
		default:
			return false;
	}
}

static void set_holding_register(struct ctrl *c, uint addr, uint val)
{
	struct port *port = c->for_port;

	switch (addr) {
		case 1:
			port->baud_rate = val * 100;
			c->need_set_port_params = true;
			break;
		case 2:
			port->parity = val;
			c->need_set_port_params = true;
			break;
		case 3:
			port->powered = val;
			c->need_set_port_params = true;
			break;
		case 4:
			port->request_timeout = val;
			c->need_set_port_params = true;
			break;
		default:
			ASSERT(0);
	}
}

static void func_read_registers(struct ctrl *c, bool holding)
{
	if (read_remains(c) < 4)
		return report_error(c, MODBUS_ERR_ILLEGAL_DATA_VALUE);

	uint start = read_u16(c);
	uint count = read_u16(c);

	uint bytes = 2*count;
	if (!body_fits(1 + bytes))
		return report_error(c, MODBUS_ERR_ILLEGAL_DATA_VALUE);

	switch (c->state) {
		case CSTATE_INIT:
			CTRL_DBG(c, "Read %s registers %u+%u", (holding ? "holding" : "input"), start, count);

			for (uint i = 0; i < count; i++)
				if (!(holding ? check_holding_register_addr : check_input_register_addr)(c, start + i))
					return report_error(c, MODBUS_ERR_ILLEGAL_DATA_ADDRESS);

			if (c->need_get_port_status) {
				usb_submit_get_port_status(c->for_port);
				c->state = CSTATE_USB_READ;
				return;
			}
			break;
		case CSTATE_USB_READ:
			break;
		default:
			ASSERT(0);
	}

	write_byte(c, bytes);
	for (uint i = 0; i < count; i++)
		write_u16(c, (holding ? get_holding_register : get_input_register)(c, start + i));

	c->state = CSTATE_DONE;
}

static void func_write_single_register(struct ctrl *c)
{
	if (read_remains(c) < 4)
		return report_error(c, MODBUS_ERR_ILLEGAL_DATA_VALUE);

	uint addr = read_u16(c);
	uint value = read_u16(c);

	switch (c->state) {
		case CSTATE_INIT:
			CTRL_DBG(c, "Write single register %u=%u", addr, value);

			if (!check_holding_register_addr(c, addr))
				return report_error(c, MODBUS_ERR_ILLEGAL_DATA_ADDRESS);

			if (!check_holding_register_write(c, addr, value))
				return report_error(c, MODBUS_ERR_SLAVE_DEVICE_FAILURE);

			set_holding_register(c, addr, value);

			if (c->need_set_port_params) {
				usb_submit_set_port_params(c->for_port);
				c->state = CSTATE_USB_WRITE;
				return;
			}
			break;
		case CSTATE_USB_WRITE:
			break;
		default:
			ASSERT(0);
	}

	write_u16(c, addr);
	write_u16(c, value);

	c->state = CSTATE_DONE;
}

static void func_write_multiple_registers(struct ctrl *c)
{
	if (read_remains(c) < 5)
		return report_error(c, MODBUS_ERR_ILLEGAL_DATA_VALUE);

	uint start = read_u16(c);
	uint count = read_u16(c);
	byte bytes = read_byte(c);

	if (read_remains(c) < bytes || bytes != 2*count)
		return report_error(c, MODBUS_ERR_ILLEGAL_DATA_VALUE);

	uint val[count];

	switch (c->state) {
		case CSTATE_INIT:
			CTRL_DBG(c, "Write multiple registers %u+%u", start, count);

			for (uint i = 0; i < count; i++) {
				if (!check_holding_register_addr(c, start + i))
					return report_error(c, MODBUS_ERR_ILLEGAL_DATA_ADDRESS);
				val[i] = read_u16(c);
			}

			for (uint i = 0; i < count; i++)
				if (!check_holding_register_write(c, start + i, val[i]))
					return report_error(c, MODBUS_ERR_SLAVE_DEVICE_FAILURE);

			for (uint i = 0; i < count; i++)
				set_holding_register(c, start + i, val[i]);

			if (c->need_set_port_params) {
				usb_submit_set_port_params(c->for_port);
				c->state = CSTATE_USB_WRITE;
				return;
			}
			break;
		case CSTATE_USB_WRITE:
			break;
		default:
			ASSERT(0);
	}

	write_u16(c, start);
	write_u16(c, count);
	c->state = CSTATE_DONE;
}

bool control_is_ready(struct box *box)
{
	// We currently process control messages one at a time
	return clist_empty(&box->control_messages_qn);
}

static void control_process_message(struct ctrl *c)
{
	// Called multiple times in different stages of message processing.
	// Each time, we read the message again.

	struct message *m = c->msg;
	c->rpos = m->request + 1;
	c->rend = m->request + m->request_size;

	m->reply[0] = m->request[0];
	m->reply[1] = m->request[1];
	c->wpos = m->reply + 2;

	uint func = read_byte(c);
	CTRL_DBG(c, "addr=%02x func=%02x state=%d", m->request[0], func, c->state);
	switch (func) {
		case MODBUS_FUNC_READ_HOLDING_REGISTERS:
			func_read_registers(c, true);
			break;
		case MODBUS_FUNC_READ_INPUT_REGISTERS:
			func_read_registers(c, false);
			break;
		case MODBUS_FUNC_WRITE_SINGLE_REGISTER:
			func_write_single_register(c);
			break;
		case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS:
			func_write_multiple_registers(c);
			break;
		default:
			report_error(c, MODBUS_ERR_ILLEGAL_FUNCTION);
	}

	CTRL_DBG(c, "new_state=%d", c->state);
	if (c->state == CSTATE_DONE) {
		m->reply_size = c->wpos - m->reply;
		msg_send_reply(m);
	}
}

void control_submit_message(struct message *m)
{
	uint slave_addr = m->request[0];
	if (slave_addr < 1 || slave_addr > NUM_PORTS) {
		msg_send_error_reply(m, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);
		return;
	}

	ASSERT(!m->ctrl);
	m->ctrl = xmalloc_zero(sizeof(struct ctrl));
	struct ctrl *c = m->ctrl;
	c->msg = m;
	c->for_port = &m->box->ports[slave_addr];
	c->state = CSTATE_INIT;
	control_process_message(c);
}

void control_usb_done(struct box *box)
{
	// Called when the USB module finishes processing of a submitted control request
	
	struct message *m = clist_head(&box->control_messages_qn);
	ASSERT(m);
	control_process_message(m->ctrl);
}
