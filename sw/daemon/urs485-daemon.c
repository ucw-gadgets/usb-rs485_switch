/*
 *	USB-RS485 Switch Daemon
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#define LOCAL_DEBUG

#include "daemon.h"

#include <ucw/conf.h>
#include <ucw/opt.h>

// Configuration

uint tcp_port_base;
uint tcp_timeout;

static struct cf_section tcp_config = {
	CF_ITEMS {
		CF_UINT("PortBase", &tcp_port_base),
		CF_UINT("Timeout", &tcp_timeout),
		CF_END
	}
};

static const struct opt_section options = {
	OPT_ITEMS {
		OPT_HELP("A daemon for controlling the USB-RS485 switch."),
		OPT_HELP("Usage: urs485-daemon [options]"),
		OPT_HELP(""),
		OPT_HELP("Options:"),
		OPT_HELP_OPTION,
		OPT_CONF_OPTIONS,
		OPT_END
	}
};

// Ports
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

int main(int argc UNUSED, char **argv)
{
	cf_def_file = "config";
	cf_declare_section("TCP", &tcp_config, 0);
	opt_parse(&options, argv+1);

	main_init();
	usb_init();
	ports_init();
	usb_init();
	sched_init();

	main_loop();
}
