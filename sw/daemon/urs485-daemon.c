/*
 *	USB-RS485 Switch Daemon
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#define LOCAL_DEBUG

#include <ucw/lib.h>
#include <ucw/mainloop.h>
#include <ucw/unaligned.h>

#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <libusb.h>

#include "usb.h"
#include "../firmware/interface.h"

#define CTRL_PORT 8
#define NUM_PORTS 9

struct message_node {
	cnode n;
	struct client *client;
	u16 client_transaction_id;
	u16 usb_message_id;
	struct port *port;
	uint frame_size;
	byte frame[2 + MODBUS_MAX_DATA_SIZE];	// As in struct urs485_message, without CRC
};

struct client {
	int id;				// ID used in messages (we use socket FDs for that)
	struct main_rec_io rio;		// TCP connection with the client
	struct port *port;
	cnode rx_messages;		// Received from the client, waiting for processing
	cnode busy_messages;		// Being processed
	cnode tx_messages;		// Processed, waiting for return to client
};

struct port {
	int port_number;
	struct main_file listen_file;
	// FIXME: Port settings
};

static struct port port[NUM_PORTS];

struct tcp_modbus_header {
	// All fields are big-endian
	u16 transaction_id;
	u16 protocol_id;		// Always 0
	u16 length;
	byte unit_id;			// Destination address
} __attribute__((packed));

#if 0	// FIXME
static struct libusb_context *usb_ctxt;
static struct libusb_device_handle *devh;

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
#endif

struct client *client_new(struct port *port, int id)
{
	struct client *c = xmalloc_zero(sizeof(*c));
	c->id = id;
	c->port = port;

	// FIXME: Initialize the rest

	return c;
}

static void client_free(struct client *client)
{
	rec_io_del(&client->rio);

	// FIXME: Delete the rest

	xfree(client);
}

static uint sk_read_handler(struct main_rec_io *rio)
{
	struct client *client = rio->data;

	if (rio->read_avail < sizeof(struct tcp_modbus_header))
		return 0;

	struct tcp_modbus_header *h = (struct tcp_modbus_header *) rio->read_buf;
	uint protocol_id = get_u16_be(&h->protocol_id);
	if (protocol_id) {
		// FIXME: Reject connection
		return ~0U;
	}
	uint len = get_u16_be(&h->length);
	if (len > 1 + MODBUS_MAX_DATA_SIZE) {
		// FIXME: Reject connection
		return ~0U;
	}
	if (rio->read_avail < sizeof(struct tcp_modbus_header) + len)
		return 0;

	DBG("Client %d: Got packet");

	struct message_node *m = xmalloc(sizeof(*m));
	m->client = client;
	m->client_transaction_id = get_u16_be(h->transaction_id);
	m->usb_message_id = 0;
	m->port = client->port;
	m->frame_size = 1 + len;
	m->frame[0] = h->unit_id;
	memcpy(m->frame + 1, rio->read_buf + sizeof(struct tcp_modbus_header), len);

	// FIXME: Enqueue the message for processing

	return sizeof(struct tcp_modbus_header) + len;
}

static int sk_notify_handler(struct main_rec_io *rio, int status)
{
	struct client *client = rio->data;

	if (status < 0) {
		switch (status) {
			case RIO_ERR_READ:
				msg(L_ERROR_R, "Client %d: Read error: %m", client->id);
				break;
			case RIO_ERR_WRITE:
				msg(L_ERROR_R, "Client %d: Write error: %m", client->id);
				break;
			default:
				msg(L_ERROR_R, "Client %d: Unknown error", client->id);
		}
		client_free(client);
		return HOOK_IDLE;
	} else if (status == RIO_EVENT_EOF) {
		msg(L_INFO_R, "Client %d: Closed connection", client->id);
		client_free(client);
		return HOOK_IDLE;
	} else {
		return HOOK_RETRY;
	}
}

static int listen_handler(struct main_file *fi)
{
	// Listening socket ready to accept a connection
	struct port *port = fi->data;

	struct sockaddr_in6 peer_addr;
	socklet_t peer_addr_len = sizeof(peer_addr);
	int sk = accept(fi->fd, &peer_addr, &peer_addr_len);
	if (sk < 0) {
		msg(L_ERROR, "Error accepting connection: %m");
		return HOOK_RETRY;
	}

	if (fcntl(sk, F_SETFL, O_NONBLOCK) < 0)
		die("Cannot set socket non-blocking: %m");

	struct client *client = conn_new(port, sk);
	struct main_rec_io *rio = &client->rio;
	rio->read_handler = sk_read_handler;
	rio->notify_handler = sk_notify_handler;
	rio->data = client;
	rio->write_throttle_read = 16384;
	// FIXME: Timeouts
	rec_io_add(rio, sk);
	rec_io_start_read(rio);

	// FIXME: Log remote address
	msg(L_INFO_R, "Client %d: New connection", client->id);

	return HOOK_RETRY;
}

static void net_init(void)
{
	// FIXME: Initialize further ports
	// FIXME: Configuration
	int sk = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
	if (sk < 0)
		die("Cannot create TCPv6 socket: %m");

	struct sockaddr_in6 sin = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(4300),
		.sin6_addr = IN6ADDR_ANY_INIT,
	};
	if (bind(sk, &sin, sizeof(sin)) < 0)
		die("Cannot bind on port 4300: %m");

	if (listen(sk, 64) < 0)
		die("Cannost listen in port 4300: %m");

	port[0].port_number = 0;
	port[0].listen_file.fd = sk;
	port[0].listen_file.read_handler = listen_handler;
	port[0].listen_file.data = &port[0];
	file_add(&port[0].listen_file);
}

int main(void)
{
	// FIXME: Options
	usb_init();
	net_init();

	main_loop();
}
