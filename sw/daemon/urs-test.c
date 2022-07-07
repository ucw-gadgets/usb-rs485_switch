/*
 *	USB-RS485 Switch Test Client
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#include <ucw/lib.h>
#include <ucw/unaligned.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <libusb.h>

static struct libusb_context *usb_ctxt;
static struct libusb_device_handle *devh;

#include "../firmware/interface.h"

static void usb_error(const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	vfprintf(stderr, msg, args);
	fputc('\n', stderr);
	va_end(args);

	if (devh) {
		libusb_close(devh);
		devh = NULL;
	}
}

static void open_device(void)
{
	int err;
	libusb_device **devlist;
	ssize_t devn = libusb_get_device_list(usb_ctxt, &devlist);
	if (devn < 0)
		die("Cannot enumerate USB devices: error %d", (int) devn);

	for (ssize_t i=0; i<devn; i++) {
		struct libusb_device_descriptor desc;
		libusb_device *dev = devlist[i];
		if (!libusb_get_device_descriptor(dev, &desc)) {
			if (desc.idVendor == URS485_USB_VENDOR && desc.idProduct == URS485_USB_PRODUCT) {
				fprintf(stderr, "Found device at usb%d.%d\n", libusb_get_bus_number(dev), libusb_get_device_address(dev));

				if (err = libusb_open(dev, &devh)) {
					usb_error("Cannot open device: error %d", err);
					goto out;
				}
				libusb_reset_device(devh);
				if (err = libusb_claim_interface(devh, 0)) {
					usb_error("Cannot claim interface: error %d", err);
					goto out;
				}

				goto out;
			}
		}
	}

out:
	libusb_free_device_list(devlist, 1);
}

int main(void)
{
	int err;
	if (err = libusb_init(&usb_ctxt))
		die("Cannot initialize libusb: error %d", err);
	// libusb_set_debug(usb_ctxt, 3);
	open_device();

	int received;
	byte resp[64];

	for (;;) {
		if (!devh) {
			fprintf(stderr, "Waiting for device to appear...\n");
			while (!devh) {
				sleep(5);
				open_device();
			}
		}

		if ((received = libusb_control_transfer(devh, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR, URS485_CONTROL_GET_CONFIG, 0, 0, resp, sizeof(resp), 1000)) < 0) {
			usb_error("Receive failed: error %d", received);
			continue;
		}
		ASSERT(received == 2);
		uint qsize = get_u16_le(resp);
		msg(L_INFO, "Config: queue size %u", qsize);

		struct urs485_port_params pp = {
			.baud_rate = 19200,
			.parity = URS485_PARITY_EVEN,
			.powered = 0,
			.request_timeout = 2000,
		};
		if ((received = libusb_control_transfer(devh, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, URS485_CONTROL_SET_PORT_PARAMS, 0, 2, (byte *) &pp, sizeof(pp), 1000)) < 0) {
			usb_error("Receive failed: error %d", received);
			continue;
		}

		for (uint i=0; i<qsize; i++) {
			byte rr[1024];

			if (err = libusb_bulk_transfer(devh, 0x82, rr, sizeof(rr), &received, 1000)) {
				usb_error("Receive failed: error %d", err);
				return 1;
			}
			printf("Receive OK: %d bytes\n", received);
		}

		for (uint i=0; i<1; i++) {
			struct urs485_message msg = {
				.port = 2,
				.frame_size = 2,
				.message_id = 0xcafe + i,
				.frame = { 42, 0x34 },
			};
			if (err = libusb_bulk_transfer(devh, 0x01, (byte *) &msg, URS485_MSGHDR_SIZE + msg.frame_size, &received, 2000)) {
				usb_error("Send failed: error %d", err);
				return 1;
			}
			printf("Send OK: %d bytes\n", received);
		}

		for (;;) {
			byte rr[1024];

			if (err = libusb_bulk_transfer(devh, 0x82, rr, sizeof(rr), &received, 5000)) {
				usb_error("Receive failed: error %d", err);
				return 1;
			}
			printf("Receive OK: %d bytes\n", received);
		}

		return 0;
	}
}
