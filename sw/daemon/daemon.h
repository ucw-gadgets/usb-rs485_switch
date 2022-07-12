/*
 *	USB-RS485 Switch Daemon
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#include <ucw/lib.h>
#include <ucw/clists.h>
#include <ucw/mainloop.h>

#include "../firmware/interface.h"
#include "modbus-proto.h"		// From stm32lib

#define NUM_PORTS 9			// 0 is the control port, 1-8 data ports

struct message {
	cnode queue_node;		// In either port->ready_messages or busy_messages
	cnode client_node;		// In one of client's lists or orphan_list
	struct box *box;
	struct client *client;		// NULL for orphaned messages (client disconnected)
	u16 client_transaction_id;
	u16 usb_message_id;
	struct port *port;
	uint request_size;
	byte request[2 + MODBUS_MAX_DATA_SIZE];	// As in struct urs485_message, without CRC
	uint reply_size;
	byte reply[2 + MODBUS_MAX_DATA_SIZE];
};

struct client {
	int id;				// ID used in messages (we use socket FDs for that)
	struct main_rec_io rio;		// TCP connection with the client
	struct box *box;
	struct port *port;
	clist rx_messages_cn;		// Received from the client, waiting for processing
	clist busy_messages_cn;		// Being processed
};

#define CLIENT_MSG(client, level, fmt, ...) msg(level, "Client %d: " fmt, client->id, ##__VA_ARGS__)
#define CLIENT_DBG(client, fmt, ...) DBG("Client %d: " fmt, client->id, ##__VA_ARGS__)

struct port {
	struct box *box;
	int port_number;
	struct main_file listen_file;
	clist ready_messages_qn;	// Ready to be sent over USB
	// FIXME: Port settings
};

struct box {				// Switch device
	struct port ports[NUM_PORTS];
	clist busy_messages_qn;		// Sent over USB, waiting for reply
	clist orphaned_messages_cn;	// Used instead of a client's list for orphaned messages
	struct main_hook sched_hook;
	u16 usb_id;
};

extern struct box only_box;		// So far we handle only one box

/* urs485-daemon.c */

extern uint tcp_port_base;
extern uint tcp_timeout;

/* client.c */

void net_init_port(struct port *port);

void msg_free(struct message *m);
void msg_send_reply(struct message *m);
void msg_send_error_reply(struct message *m, enum modbus_error err);

/* usb.c */

void usb_init(void);
bool usb_is_ready(struct box *box);
void usb_submit_message(struct message *m);
