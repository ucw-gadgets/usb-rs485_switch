/*
 *	USB-RS485 Switch Daemon -- USB Communication
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#define LOCAL_DEBUG

#include "daemon.h"
#include "mainloop-usb.h"

#include <libusb.h>
#include <stdarg.h>
#include <stdio.h>
#include <ucw/stkstring.h>
#include <ucw/unaligned.h>

// States of USB connection
enum usb_state {
	USTATE_INIT,
	USTATE_GET_DEV_CONFIG,
	USTATE_SET_PORT_CONFIG,
	USTATE_WORKING = USTATE_SET_PORT_CONFIG + 8,
	USTATE_BROKEN,
};

struct usb_context {
	struct box *box;
	const char *switch_name;		// Used in debug messages
	enum usb_state state;
	int bus, dev;				// -1 if device already unplugged
	struct main_timer connect_timer;
	u16 last_id;				// Last ID assigned to a message

	struct libusb_device_handle *devh;
	struct libusb_transfer *ctrl_transfer, *rx_transfer, *tx_transfer;
	bool ctrl_in_flight, rx_in_flight, tx_in_flight;

	byte ctrl_buffer[256];

	struct urs485_message rx_message, tx_message;
	uint tx_window;

	// Which control message is being processed
	enum urs485_control_request ctrl_current;
	struct port *ctrl_port;
};

struct hotplug_request {
	cnode n;
	libusb_device *device;			// referenced
	int bus, dev;
	char name[16];
	bool is_connect;
};

static clist hotplug_request_list;

#define USB_MSG(usb_ctx, level, fmt, ...) msg(level, "USB(%s): " fmt, usb_ctx->switch_name, ##__VA_ARGS__)
#define USB_DBG(usb_ctx, fmt, ...) DBG("USB(%s): " fmt, usb_ctx->switch_name, ##__VA_ARGS__)

#define HR_MSG(hp_req, level, fmt, ...) msg(level, "%s: " fmt, hp_req->name, ##__VA_ARGS__)
#define HR_DBG(hp_req, fmt, ...) DBG("%s: " fmt, hp_req->name, ##__VA_ARGS__)

static void handle_hotplug(void);
static void startup_scheduler(struct usb_context *u);
static void rx_init(struct usb_context *u);

static void FORMAT_CHECK(printf,2,3) usb_error(struct usb_context *u, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	const char *formatted_msg = stk_vprintf(fmt, args);
	va_end(args);

	USB_MSG(u, L_ERROR, "%s", formatted_msg);
	u->state = USTATE_BROKEN;
}

bool usb_is_ready(struct box *box)
{
	struct usb_context *u = box->usb;

	// If the switch is not ready, we generate internal errors for all messages
	if (!u || u->state != USTATE_WORKING)
		return true;

	return (!u->tx_in_flight && u->tx_window > 0);
}

static void tx_callback(struct libusb_transfer *xfer)
{
	struct usb_context *u = xfer->user_data;

	USB_DBG(u, "Bulk TX done (status=%d, len=%d/%d)", xfer->status, xfer->actual_length, xfer->length);
	u->tx_in_flight = false;

	if (xfer->status != LIBUSB_TRANSFER_COMPLETED)
		usb_error(u, "Bulk TX transfer failed with status %d", xfer->status);
}

static void usb_gen_id(struct message *m)
{
	struct box *box = m->box;
	struct usb_context *u = box->usb;

	for (;;) {
		u->last_id++;
		bool used = false;
		CLIST_FOR_EACH(struct message *, n, box->busy_messages_qn)
			if (n != m && n->usb_message_id == u->last_id) {
				used = true;
				break;
			}
		if (!used) {
			m->usb_message_id = u->last_id;
			return;
		}
	}
}

void usb_submit_message(struct message *m)
{
	// To be called only after usb_is_ready() returns true

	struct usb_context *u = m->box->usb;
	if (u->state != USTATE_WORKING) {
		msg_send_error_reply(m, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);
		return;
	}

	ASSERT(!u->tx_in_flight && u->tx_window > 0);
	u->tx_window--;

	usb_gen_id(m);

	struct urs485_message *tm = &u->tx_message;
	tm->port = m->port->port_number - 1;
	tm->frame_size = m->request_size;
	put_u16_le(&tm->message_id, m->usb_message_id);
	memcpy(tm->frame, m->request, m->request_size);

	USB_DBG(u, "TX: port=%d, frame_size=%d, msg_id=%04x", tm->port, tm->frame_size, m->usb_message_id);

	uint tx_size = offsetof(struct urs485_message, frame) + m->request_size;
	libusb_fill_bulk_transfer(u->tx_transfer, u->devh, 0x01, (unsigned char *) &u->tx_message, tx_size, tx_callback, u, 5000);

	int err;
	if (err = libusb_submit_transfer(u->tx_transfer))
		usb_error(u, "Cannot submit bulk TX transfer: error %d", err);
	else
		u->tx_in_flight = true;
}

static void rx_process_msg(struct usb_context *u)
{
	struct urs485_message *rm = &u->rx_message;
	u16 msg_id = get_u16_le(&rm->message_id);
	USB_DBG(u, "RX: port=%d, frame_size=%d, msg_id=%04x", rm->port, rm->frame_size, msg_id);

	if (rm->port == 0xff) {
		// It was a window open message
		return;
	}

	CLIST_FOR_EACH(struct message *, m, u->box->busy_messages_qn) {
		if (m->usb_message_id == msg_id) {
			m->reply_size = rm->frame_size;
			ASSERT(m->reply_size < sizeof(m->reply));
			memcpy(m->reply, rm->frame, m->reply_size);
			msg_send_reply(m);
			return;
		}
	}

	USB_MSG(u, L_WARN, "Switch replied to an unknown message #%04x", msg_id);
}

static void rx_callback(struct libusb_transfer *xfer)
{
	struct usb_context *u = xfer->user_data;

	USB_DBG(u, "Bulk RX done (status=%d, len=%d/%d)", xfer->status, xfer->actual_length, xfer->length);
	u->rx_in_flight = false;

	if (xfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
		// XXX: Later, we can turn off the RX pipe if there are no packets in flight
		rx_init(u);
		return;
	}

	if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
		usb_error(u, "Bulk RX transfer failed with status %d", xfer->status);
		return;
	}

	rx_process_msg(u);

	u->tx_window++;
	rx_init(u);
}

static void rx_init(struct usb_context *u)
{
	ASSERT(!u->rx_in_flight);

	libusb_fill_bulk_transfer(u->rx_transfer, u->devh, 0x82, (unsigned char *) &u->rx_message, sizeof(u->rx_message), rx_callback, u, 5000);

	int err;
	if (err = libusb_submit_transfer(u->rx_transfer))
		usb_error(u, "Cannot submit bulk RX transfer: error %d", err);
	else
		u->rx_in_flight = true;
}

static void ctrl_callback(struct libusb_transfer *xfer)
{
	struct usb_context *u = xfer->user_data;

	USB_DBG(u, "Ctrl done (status=%d, len=%d/%d)", xfer->status, xfer->actual_length, xfer->length);
	u->ctrl_in_flight = false;

	if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
		usb_error(u, "Control transfer failed with status %d", xfer->status);
		return;
	}

	switch (u->ctrl_current) {
		case URS485_CONTROL_GET_CONFIG: {
			struct urs485_config *cf = (struct urs485_config *)(u->ctrl_buffer + 8);
			USB_DBG(u, "max_in_flight=%d", get_u16(&cf->max_in_flight));
			break;
		}
		case URS485_CONTROL_GET_PORT_STATUS: {
			struct urs485_port_status *ps = (struct urs485_port_status *)(u->ctrl_buffer + 8);
			u->ctrl_port->current_sense = get_u16_le(&ps->current_sense);
			break;
		}
		default: ;
	}

	u->ctrl_current = 0;
	u->ctrl_port = NULL;

	ASSERT(u->state != USTATE_INIT);
	if (u->state < USTATE_WORKING) {
		u->state++;
		startup_scheduler(u);
	} else {
		control_usb_done(u->box);
	}
}

static void usb_submit_ctrl(struct usb_context *u, struct port *port, enum urs485_control_request req, bool direction_out, uint data_size)
{
	u->ctrl_current = req;
	u->ctrl_port = port;

	libusb_fill_control_setup(u->ctrl_buffer,
		(direction_out ? LIBUSB_ENDPOINT_OUT : LIBUSB_ENDPOINT_IN) | LIBUSB_REQUEST_TYPE_VENDOR,
		req,
		0,
		(port ? port->port_number : 0),
		data_size);

	libusb_fill_control_transfer(u->ctrl_transfer, u->devh, u->ctrl_buffer, ctrl_callback, u, 5000);

	int err;
	if (err = libusb_submit_transfer(u->ctrl_transfer))
		usb_error(u, "Cannot submit control transfer: error %d", err);
	else
		u->ctrl_in_flight = true;
}

void usb_submit_get_port_status(struct port *port)
{
	struct usb_context *u = port->box->usb;
	USB_DBG(u, "GET_PORT_STATUS on port %d", port->port_number);
	usb_submit_ctrl(u, port, URS485_CONTROL_GET_PORT_STATUS, false, sizeof(struct urs485_port_status));
}

void usb_submit_set_port_params(struct port *port)
{
	struct usb_context *u = port->box->usb;
	USB_DBG(u, "SET_PORT_PARAMS on port %d", port->port_number);

	struct urs485_port_params *pp = (struct urs485_port_params *)(u->ctrl_buffer + 8);
	put_u32_le(&pp->baud_rate, port->baud_rate);
	pp->parity = port->parity;
	pp->powered = port->powered;
	put_u16_le(&pp->request_timeout, port->request_timeout);

	usb_submit_ctrl(u, port, URS485_CONTROL_SET_PORT_PARAMS, true, sizeof(struct urs485_port_params));
}

static void startup_scheduler(struct usb_context *u)
{
	// State machine for the initialization sequence
	ASSERT(u->state != USTATE_INIT);

	if (u->state == USTATE_GET_DEV_CONFIG) {
		USB_DBG(u, "Init: Get device config");
		usb_submit_ctrl(u, NULL, URS485_CONTROL_GET_CONFIG, false, sizeof(struct urs485_config));
	} else if (u->state < USTATE_WORKING) {
		uint port_number = u->state - USTATE_SET_PORT_CONFIG;
		USB_DBG(u, "Init: Setting up port %d", port_number);
		usb_submit_set_port_params(&u->box->ports[port_number]);
	} else {
		USB_DBG(u, "USB init: Done");
		rx_init(u);
	}
}

static void connect_handler(struct main_timer *timer)
{
	struct usb_context *u = timer->data;
	int err;

	timer_del(timer);

	libusb_reset_device(u->devh);

	if (err = libusb_claim_interface(u->devh, 0)) {
		usb_error(u, "Cannot claim interface: error %d", err);
		return;
	}

	ASSERT(!u->ctrl_in_flight && !u->rx_in_flight && !u->tx_in_flight);
	u->tx_window = 0;

	u->state = USTATE_GET_DEV_CONFIG;
	startup_scheduler(u);
}

static void check_if_broken(struct usb_context *u)
{
	// USB devices cannot be closed from libusb callbacks, so we handle it here
	if (u->state != USTATE_BROKEN)
		return;

	// If there are still in-flight transfers, wait for them to complete
	if (u->ctrl_in_flight || u->rx_in_flight || u->tx_in_flight)
		return;

	USB_DBG(u, "Leaving broken device");
	u->state = USTATE_INIT;

	// Flush all in-progress messages
	struct box *box = u->box;
	struct message *m;
	while (m = clist_head(&box->busy_messages_qn))
		msg_send_error_reply(m, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);
	while (m = clist_head(&box->control_messages_qn))
		msg_send_error_reply(m, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);

	if (u->bus < 0) {
		// Device already unplugged
		USB_DBG(u, "Removing device");

		timer_del(&u->connect_timer);

		if (u->devh) {
			libusb_close(u->devh);
			u->devh = NULL;
		}

		libusb_free_transfer(u->ctrl_transfer);
		libusb_free_transfer(u->rx_transfer);
		libusb_free_transfer(u->tx_transfer);

		xfree(u);
		box->usb = NULL;
	} else {
		// Re-connect later
		timer_add_rel(&u->connect_timer, 5000);
	}
}

static int usb_hook_handler(struct main_hook *hook UNUSED)
{
	handle_hotplug();
	CLIST_FOR_EACH(struct box *, b, box_list)
		if (b->usb)
			check_if_broken(b->usb);
	return HOOK_IDLE;
}

static struct main_hook usb_hook;

static struct box *find_box(const char *serial)
{
	CLIST_FOR_EACH(struct box *, b, box_list)
		if (!b->cf->serial || !strcmp(b->cf->serial, serial))
			return b;
	return NULL;
}

static void hotplug_connect(struct hotplug_request *hr)
{
	HR_MSG(hr, L_INFO, "Connected");

	// We might get duplicate events, so ignore the event if the device is already known
	CLIST_FOR_EACH(struct box *, b, box_list) {
		struct usb_context *u = b->usb;
		if (u && u->bus == hr->bus && u->dev == hr->dev)
			return;
	}

	struct libusb_device_descriptor desc;
	int err;

	if (err = libusb_get_device_descriptor(hr->device, &desc)) {
		HR_MSG(hr, L_ERROR, "Cannot read descriptors: error %d", err);
		return;
	}

	libusb_device_handle *devh;
	if (err = libusb_open(hr->device, &devh)) {
		HR_MSG(hr, L_ERROR, "Cannot open device: error %d", err);
		return;
	}

	byte serial[64];
	if ((err = libusb_get_string_descriptor_ascii(devh, desc.iSerialNumber, serial, sizeof(serial))) < 0) {
		HR_MSG(hr, L_ERROR, "Cannot get serial number: error %d", err);
		goto out;
	}

	struct box *box = find_box((const char *) serial);
	if (!box) {
		HR_MSG(hr, L_ERROR, "No switch configuration matches serial number %s", serial);
		goto out;
	}
	if (box->usb) {
		HR_MSG(hr, L_ERROR, "Switch configuration for serial number %s is already in use", serial);
		goto out;
	}
	HR_MSG(hr, L_INFO, "Configured as switch %s (serial number %s)", box->cf->name, serial);

	struct usb_context *u = xmalloc_zero(sizeof(*u));
	u->box = box;
	u->switch_name = box->cf->name;
	box->usb = u;

	u->state = USTATE_INIT;
	u->bus = hr->bus;
	u->dev = hr->dev;
	u->devh = devh;

	u->ctrl_transfer = libusb_alloc_transfer(0);
	u->rx_transfer = libusb_alloc_transfer(0);
	u->tx_transfer = libusb_alloc_transfer(0);
	ASSERT(u->ctrl_transfer && u->rx_transfer && u->tx_transfer);

	u->connect_timer.handler = connect_handler;
	u->connect_timer.data = u;
	timer_add_rel(&u->connect_timer, 0);
	return;

out:
	libusb_close(devh);
}

static void hotplug_disconnect(struct hotplug_request *hr)
{
	HR_MSG(hr, L_INFO, "Device disconnected");

	CLIST_FOR_EACH(struct box *, b, box_list) {
		struct usb_context *u = b->usb;
		if (u && u->bus == hr->bus && u->dev == hr->dev) {
			u->state = USTATE_BROKEN;
			u->bus = u->dev = -1;
			return;
		}
	}
}

static void handle_hotplug(void)
{
	struct hotplug_request *hr;
	while (hr = clist_remove_head(&hotplug_request_list)) {
		if (hr->is_connect)
			hotplug_connect(hr);
		else
			hotplug_disconnect(hr);
		libusb_unref_device(hr->device);
		xfree(hr);
	}
}

static int hotplug_callback(libusb_context *ctx UNUSED, libusb_device *device, libusb_hotplug_event event, void *user_data UNUSED)
{
	// Called asynchronously, need to defer to the main loop

	if (event != LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED && event != LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT)
		return 0;

	struct hotplug_request *hr = xmalloc_zero(sizeof(*hr));
	hr->device = libusb_ref_device(device);
	hr->bus = libusb_get_bus_number(device);
	hr->dev = libusb_get_device_address(device);
	snprintf(hr->name, sizeof(hr->name), "usb%d.%d", hr->bus, hr->dev);
	hr->is_connect = (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED);
	clist_add_tail(&hotplug_request_list, &hr->n);

	HR_DBG(hr, "Scheduled hotplug event");
	return 0;
}

void usb_init(void)
{
	usb_init_mainloop();

	clist_init(&hotplug_request_list);
	usb_hook.handler = usb_hook_handler;
	hook_add(&usb_hook);

	int err;
	if (err = libusb_hotplug_register_callback(usb_ctx,
		LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
		LIBUSB_HOTPLUG_ENUMERATE,
		URS485_USB_VENDOR,
		URS485_USB_PRODUCT,
		LIBUSB_HOTPLUG_MATCH_ANY,
		hotplug_callback,
		NULL,			// no user data
		NULL			// no need to store callback handle
		))
		die("Cannot register libusb hotplug callback");
}
