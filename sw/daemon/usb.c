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

static struct main_timer usb_connect_timer;

static struct libusb_device_handle *usb_devh;

static struct libusb_transfer *ctrl_transfer, *rx_transfer, *tx_transfer;

static byte ctrl_buffer[256];

static struct urs485_message rx_message, tx_message;
static byte rx_in_flight, tx_in_flight;
static uint tx_window;

// States of USB connection
static enum usb_state {
	USTATE_IDLE,
	USTATE_GET_DEV_CONFIG,
	USTATE_SET_PORT_CONFIG,
	USTATE_WORKING = USTATE_SET_PORT_CONFIG + 8,
	USTATE_BROKEN,
} usb_state;

static void startup_scheduler(void);
static void rx_init(void);

static void usb_error(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	const char *formatted_msg = stk_vprintf(fmt, args);
	va_end(args);

	msg(L_ERROR, "USB: %s", formatted_msg);
	usb_state = USTATE_BROKEN;
}

bool usb_is_ready(void)
{	
	// If there is no device connect, we generate internal errors for all messages
	if (!usb_devh)
		return true;

	return false;	// FIXME
}

void usb_submit_message(struct message *m)
{
	// To be called only after usb_is_ready() returns true

	if (!usb_devh) {
		msg_send_error_reply(m, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);
		return;
	}

	// FIXME
}

static void rx_callback(struct libusb_transfer *xfer)
{
	DBG("USB: Bulk RX done (status=%d, len=%d/%d)", xfer->status, xfer->actual_length, xfer->length);
	rx_in_flight = false;

	if (xfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
		// XXX: Later, we can turn off the RX pipe if there are no packets in flight
		rx_init();
		return;
	}

	if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
		usb_error("Bulk RX transfer failed with status %d", xfer->status);
		return;
	}

	struct urs485_message *rm = &rx_message;
	DBG("RX: port=%d, frame_size=%d, msg_id=%04x", rm->port, rm->frame_size, rm->message_id);

	tx_window++;
	DBG("TX: Current window is %d", tx_window);
	rx_init();
}

static void rx_init(void)
{
	ASSERT(!rx_in_flight);

	libusb_fill_bulk_transfer(rx_transfer, usb_devh, 0x82, (unsigned char *) &rx_message, sizeof(rx_message), rx_callback, NULL, 5000);

	int err;
	if (err = libusb_submit_transfer(rx_transfer))
		usb_error("Cannot submit bulk RX transfer: error %d");
	else
		rx_in_flight = true;
}

static void ctrl_callback(struct libusb_transfer *xfer)
{
	DBG("USB: Ctrl done (status=%d, len=%d/%d)", xfer->status, xfer->actual_length, xfer->length);
	if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
		usb_error("Control transfer failed with status %d", xfer->status);
		return;
	}

	ASSERT(usb_state != USTATE_IDLE && usb_state < USTATE_WORKING);
	if (usb_state == USTATE_GET_DEV_CONFIG) {
		struct urs485_config *cf = (struct urs485_config *)(ctrl_buffer + 8);
		DBG("USB: max_in_flight=%d", get_u16(&cf->max_in_flight));
	}
	usb_state++;
	startup_scheduler();
}

static void startup_scheduler(void)
{
	// State machine for the initialization sequence
	ASSERT(usb_state != USTATE_IDLE);

	if (usb_state == USTATE_GET_DEV_CONFIG) {
		DBG("USB init: Get device config");
		libusb_fill_control_setup(ctrl_buffer, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR, URS485_CONTROL_GET_CONFIG, 0, 0, sizeof(struct urs485_config));
	} else if (usb_state < USTATE_WORKING) {
		// FIXME: Replace by proper port configuration
		uint port_number = usb_state - USTATE_SET_PORT_CONFIG;
		DBG("USB init: Setting up port %d", port_number);

		struct urs485_port_params *pp = (struct urs485_port_params *)(ctrl_buffer + 8);
		put_u32_le(&pp->baud_rate, 19200);
		pp->parity = URS485_PARITY_EVEN;
		pp->powered = 0;
		put_u16_le(&pp->request_timeout, 5000);

		libusb_fill_control_setup(ctrl_buffer, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, URS485_CONTROL_SET_PORT_PARAMS, 0, port_number, sizeof(struct urs485_port_params));
	} else {
		DBG("USB init: Done");
		rx_init();
		return;
	}

	libusb_fill_control_transfer(ctrl_transfer, usb_devh, ctrl_buffer, ctrl_callback, NULL, 5000);
	int err;
	if (err = libusb_submit_transfer(ctrl_transfer))
		usb_error("Cannot submit control transfer: error %d");
}

static void usb_connect_handler(struct main_timer *timer)
{
	static bool prev_found = true;
	int err;
	libusb_device **devlist;
	ssize_t devn = libusb_get_device_list(usb_ctx, &devlist);
	if (devn < 0)
		die("Cannot enumerate USB devices: error %d", (int) devn);

	for (ssize_t i=0; i<devn; i++) {
		struct libusb_device_descriptor desc;
		libusb_device *dev = devlist[i];
		if (!libusb_get_device_descriptor(dev, &desc)) {
			if (desc.idVendor == URS485_USB_VENDOR && desc.idProduct == URS485_USB_PRODUCT) {
				// FIXME: Match serial number (so that we can have one daemon per device)
				msg(L_INFO, "Found device at usb%d.%d", libusb_get_bus_number(dev), libusb_get_device_address(dev));
				prev_found = true;

				if (err = libusb_open(dev, &usb_devh)) {
					usb_error("Cannot open device: error %d", err);
					goto out;
				}

				libusb_reset_device(usb_devh);

				if (err = libusb_claim_interface(usb_devh, 0)) {
					usb_error("Cannot claim interface: error %d", err);
					goto out;
				}

				// FIXME: Initialize settings of all ports
				// FIXME: Reset queues

				timer_del(timer);

				usb_state = USTATE_GET_DEV_CONFIG;
				startup_scheduler();
				goto out;
			}
		}
	}

	if (prev_found)
		msg(L_INFO, "Waiting for USB device to appear");
	prev_found = false;
	timer_add_rel(timer, 5000);
out:
	libusb_free_device_list(devlist, 1);
}

static int broken_handler(struct main_hook *hook UNUSED)
{
	// USB devices cannot be closed from libusb callbacks, so we handle it here
	if (usb_state != USTATE_BROKEN)
		return HOOK_IDLE;

	// If there are still in-flight transfers, wait for them to complete
	if (rx_in_flight || tx_in_flight)
		return HOOK_IDLE;

	DBG("USB: Closing broken device");
	usb_state = USTATE_IDLE;

	if (usb_devh) {
		libusb_close(usb_devh);
		usb_devh = NULL;
	}

	// Flush all in-progress messages
	struct message *m;
	while (m = clist_head(&busy_messages_qn))
		msg_send_error_reply(m, MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE);

	timer_add_rel(&usb_connect_timer, 5000);
	return HOOK_IDLE;
}

static struct main_hook broken_hook;

void usb_init(void)
{
	usb_init_mainloop();

	ctrl_transfer = libusb_alloc_transfer(0);
	rx_transfer = libusb_alloc_transfer(0);
	tx_transfer = libusb_alloc_transfer(0);
	ASSERT(ctrl_transfer && rx_transfer && tx_transfer);

	broken_hook.handler = broken_handler;
	hook_add(&broken_hook);

	usb_connect_timer.handler = usb_connect_handler;
	timer_add_rel(&usb_connect_timer, 0);
}
