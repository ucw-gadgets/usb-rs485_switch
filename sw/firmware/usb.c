/*
 *	USB-RS485 Switch -- USB Interface
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "firmware.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/usb/usbd.h>

#include <string.h>

static usbd_device *usbd_dev;
static bool usb_configured;
static u32 usb_generation;		// Incremented on each bus reset

#ifdef DEBUG_USB
#define DEBUG(msg, ...) debug_printf("USB: " msg, ## __VA_ARGS__)
#else
#define DEBUG(...) do { } while (0)
#endif

/*** Descriptors ***/

enum usb_string {
	STR_MANUFACTURER = 1,
	STR_PRODUCT,
	STR_SERIAL,
};

static const char *usb_strings[] = {
	"United Computer Wizards",
	"USB-RS485 Switch",
	serial_number,
};

static const struct usb_device_descriptor device = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFF,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = URS485_USB_VENDOR,
	.idProduct = URS485_USB_PRODUCT,
	.bcdDevice = URS485_USB_VERSION,
	.iManufacturer = STR_MANUFACTURER,
	.iProduct = STR_PRODUCT,
	.iSerialNumber = STR_SERIAL,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor endpoints[] = {{
	// Bulk end-point for sending requests
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	// Bulk end-point for receiving replies
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
	.endpoint = endpoints,
};

static const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x0100,
};

static const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 0,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}, {
	.num_altsetting = 1,
	.altsetting = &dfu_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 100,	// multiplied by 2 mA
	.interface = ifaces,
};

/*** Control endpoint ***/

static uint8_t usbd_control_buffer[64];

static void dfu_detach_complete(usbd_device *dev UNUSED, struct usb_setup_data *req UNUSED)
{
	// Reset to bootloader, which implements the rest of DFU
	debug_printf("Switching to DFU\n");
	debug_flush();
	scb_reset_core();
}

static enum usbd_request_return_codes dfu_control_cb(usbd_device *dev UNUSED,
	struct usb_setup_data *req,
	uint8_t **buf UNUSED,
	uint16_t *len UNUSED,
	void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	if (req->bmRequestType != 0x21 || req->bRequest != DFU_DETACH)
		return USBD_REQ_NOTSUPP;

	*complete = dfu_detach_complete;
	return USBD_REQ_HANDLED;
}

static enum usbd_request_return_codes control_cb(
	usbd_device *dev UNUSED,
	struct usb_setup_data *req,
	uint8_t **buf,
	uint16_t *len,
	void (**complete)(usbd_device *dev, struct usb_setup_data *req) UNUSED)
{
	uint index = req->wIndex;

	if (req->bmRequestType == (USB_REQ_TYPE_IN | USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_DEVICE)) {
		DEBUG("Control request IN %02x (index=%d, len=%d)\n", req->bRequest, index, *len);

		const byte *reply = NULL;
		uint reply_len = 0;

		switch (req->bRequest) {
			case URS485_CONTROL_GET_CONFIG:
				reply = (const byte *) &global_config;
				reply_len = sizeof(global_config);
				break;
			case URS485_CONTROL_GET_PORT_STATUS:
				if (index >= 8)
					return USBD_REQ_NOTSUPP;
				reply = (const byte *) &ports[index].status;
				reply_len = sizeof(struct urs485_port_status);
				break;
			case URS485_CONTROL_GET_POWER_STATUS:
				reply = (const byte *) &power_status;
				reply_len = sizeof(power_status);
				break;
			default:
				return USBD_REQ_NOTSUPP;
		}

		uint n = MIN(*len, reply_len);
		memcpy(*buf, reply, n);
		*len = n;
		return USBD_REQ_HANDLED;
	} else if (req->bmRequestType == (USB_REQ_TYPE_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_DEVICE)) {
		DEBUG("Control request OUT %02x (index=%d, len=%d)\n", req->bRequest, index, *len);

		switch (req->bRequest) {
			case URS485_CONTROL_SET_PORT_PARAMS:
				if (index >= 8)
					return USBD_REQ_NOTSUPP;
				if (*len != sizeof(struct urs485_port_params))
					return USBD_REQ_NOTSUPP;
				struct urs485_port_params new_params;
				memcpy(&new_params, *buf, sizeof(struct urs485_port_params));
				if (!set_port_params(index, &new_params))
					return USBD_REQ_NOTSUPP;
				break;
			case URS485_CONTROL_RESET_STATS:
				if (index >= 8)
					return USBD_REQ_NOTSUPP;
				if (*len != 0)
					return USBD_REQ_NOTSUPP;
				reset_port_stats(index);
				break;

			default:
				return USBD_REQ_NOTSUPP;
		}

		return USBD_REQ_HANDLED;
	} else {
		return USBD_REQ_NOTSUPP;
	}
}

/*** Bulk endpoints ***/

static struct message_node *usb_rx_msg;
static uint usb_rx_pos;
static byte usb_rx_buffer[64];

static struct message_node *usb_tx_msg;
static uint usb_tx_pos;
static bool usb_tx_in_flight;

static void ep01_cb(usbd_device *dev, uint8_t ep UNUSED)
{
	// We received a frame from the USB host
	uint len = usbd_ep_read_packet(dev, 0x01, usb_rx_buffer, sizeof(usb_rx_buffer));
	byte *pos = usb_rx_buffer;
	DEBUG("Host sent %u bytes\n", len);

	while (len) {
		if (!usb_rx_msg) {
			usb_rx_msg = queue_get(&idle_queue);
			if (!usb_rx_msg) {
				DEBUG("No receive buffer available\n");
				// The only chance to signal error is to stall the pipes
				usbd_ep_stall_set(dev, 0x01, 1);
				usbd_ep_stall_set(dev, 0x82, 1);
				return;
			}
			usb_rx_msg->usb_generation = usb_generation;
			usb_rx_pos = 0;
		}

		uint goal = (usb_rx_pos < URS485_MSGHDR_SIZE) ? URS485_MSGHDR_SIZE : URS485_MSGHDR_SIZE + usb_rx_msg->msg.frame_size;
		uint want = goal - usb_rx_pos;
		if (want > 0) {
			want = MIN(want, len);
			memcpy((byte *) &usb_rx_msg->msg + usb_rx_pos, pos, want);
			usb_rx_pos += want;
			pos += want;
			len -= want;
		}

		if (usb_rx_pos >= URS485_MSGHDR_SIZE && usb_rx_pos == URS485_MSGHDR_SIZE + usb_rx_msg->msg.frame_size) {
			DEBUG("Received message #%04x of %u bytes\n", usb_rx_msg->msg.message_id, usb_rx_pos);
			got_msg_from_usb(usb_rx_msg);
			usb_rx_msg = NULL;
		}
	}
}

static void ep82_kick(void)
{
	if (!usb_configured || usb_tx_in_flight)
		return;

	if (!usb_tx_msg) {
		usb_tx_msg = queue_get(&done_queue);
		if (!usb_tx_msg)
			return;
		struct urs485_message *m = &usb_tx_msg->msg;
		if (usb_tx_msg->usb_generation == usb_generation) {
			DEBUG("Sending message #%04x\n", m->message_id);
		} else {
			DEBUG("Flushing previous-generation message\n");
			m->port = 0xff;
			m->frame_size = 0;
			m->message_id = 0;
		}
		usb_tx_pos = 0;
	}

	uint goal = URS485_MSGHDR_SIZE + usb_tx_msg->msg.frame_size;
	uint len = MIN(goal - usb_tx_pos, 64);
	usbd_ep_write_packet(usbd_dev, 0x82, (const byte *) &usb_tx_msg->msg + usb_tx_pos, len);
	usb_tx_in_flight = true;
	usb_tx_pos += len;

	if (usb_tx_pos == goal) {
		DEBUG("Sent\n");
		queue_put(&idle_queue, usb_tx_msg);
		usb_tx_msg = NULL;
	}
}

static void ep82_cb(usbd_device *dev UNUSED, uint8_t ep UNUSED)
{
	// We completed sending a frame to the USB host
	usb_tx_in_flight = false;
	ep82_kick();
}

/*** Main ***/

static void set_config_cb(usbd_device *dev, uint16_t wValue UNUSED)
{
	usbd_register_control_callback(
		dev,
		USB_REQ_TYPE_VENDOR,
		USB_REQ_TYPE_TYPE,
		control_cb);
	usbd_register_control_callback(
		dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		dfu_control_cb);
	usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, ep01_cb);
	usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, ep82_cb);
	usb_configured = true;

	// Increment USB generation and send all idle messages to the client,
	// so that it can increase its send window.
	usb_generation++;
	struct message_node *n;
	while (n = queue_get(&idle_queue))
		queue_put(&done_queue, n);

	// If there were in-progress transfers, cancel them
	if (usb_rx_msg) {
		queue_put(&done_queue, usb_rx_msg);
		usb_rx_msg = NULL;
	}
	if (usb_tx_msg) {
		queue_put(&done_queue, usb_tx_msg);
		usb_tx_msg = NULL;
	}
	usb_tx_in_flight = false;
}

static void reset_cb(void)
{
	DEBUG("Reset\n");
	usb_configured = false;
}

static volatile bool usb_event_pending;

void usb_lp_can_rx0_isr(void)
{
	/*
	 *  We handle USB in the main loop to avoid race conditions between
	 *  USB interrupts and other code. However, we need an interrupt to
	 *  up the main loop from sleep.
	 *
	 *  We set up only the low-priority ISR, because high-priority ISR handles
	 *  only double-buffered bulk transfers and isochronous transfers.
	 */
	nvic_disable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	usb_event_pending = 1;
}

void usb_init(void)
{
	// Disable USB Isolator for 100 ms and then turn it on
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
	gpio_clear(GPIOB, GPIO0);
	delay_ms(100);
	gpio_set(GPIOB, GPIO0);

	usbd_dev = usbd_init(
		&st_usbfs_v1_usb_driver,
		&device,
		&config,
		usb_strings,
		ARRAY_SIZE(usb_strings),
		usbd_control_buffer,
		sizeof(usbd_control_buffer)
	);
	usbd_register_reset_callback(usbd_dev, reset_cb);
	usbd_register_set_config_callback(usbd_dev, set_config_cb);
	usb_event_pending = 1;
}

void usb_loop(void)
{
	if (usb_event_pending) {
		usbd_poll(usbd_dev);
		usb_event_pending = 0;
		nvic_clear_pending_irq(NVIC_USB_LP_CAN_RX0_IRQ);
		nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	}

	ep82_kick();
}
