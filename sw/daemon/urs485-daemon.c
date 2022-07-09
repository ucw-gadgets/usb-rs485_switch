/*
 *	USB-RS485 Switch Daemon
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#define LOCAL_DEBUG

#include "daemon.h"

// FIXME: Audit includes
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static struct port ports[NUM_PORTS];

// Global message queues
clist busy_messages_qn;
clist orphaned_messages_cn;

static struct main_hook sched_hook;

static struct message *sched_next_msg(void)
{
	// Round-robin on ports. Replace by a better scheduler one day.
	static int robin = 1;

	for (int i=0; i<NUM_PORTS-1; i++) {
		robin = robin + 1;
		if (robin == NUM_PORTS)
			robin = 1;
		struct port *port = &ports[robin];
		struct message *m;
		if (m = clist_remove_head(&port->ready_messages_qn)) {
			clist_add_tail(&busy_messages_qn, &m->queue_node);
			return m;
		}
	}

	return NULL;
}

static int sched_handler(struct main_hook *hook UNUSED)
{
	struct message *m;

	// Messages on the control port are processed immediately
	while (m = clist_head(&ports[0].ready_messages_qn)) {
		// FIXME
		msg_send_error_reply(m, MODBUS_ERR_ILLEGAL_FUNCTION);
	}

	// Send messages to USB
	while (usb_is_ready()) {
		struct message *m = sched_next_msg();
		if (!m)
			break;
		usb_submit_message(m);
	}

	return HOOK_IDLE;
}

static void sched_init(void)
{
	sched_hook.handler = sched_handler;
	hook_add(&sched_hook);
}

static void port_init(int index)
{
	struct port *port = &ports[index];

	port->port_number = index;
	clist_init(&port->ready_messages_qn);

	net_init_port(port);
}

static void ports_init(void)
{
	clist_init(&busy_messages_qn);
	clist_init(&orphaned_messages_cn);

	for (int i=0; i<NUM_PORTS; i++)
		port_init(i);
}

int main(void)
{
	// FIXME: Options
	main_init();
	usb_init();
	ports_init();
	usb_init();
	sched_init();

	main_loop();
}
