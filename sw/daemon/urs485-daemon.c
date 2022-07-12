/*
 *	USB-RS485 Switch Daemon
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#include "daemon.h"

#include <ucw/conf.h>
#include <ucw/opt.h>

/*** Configuration ***/

clist switch_configs;

uint tcp_timeout;

static char *switch_commit(void *s_)
{
	struct switch_config *s = s_;
	if (!s->name)
		return "Every switch must have a Name";
	if (!s->tcp_port_base)
		return "Every switch must have a TCPPortBase";
	return NULL;
}

static struct cf_section switch_config = {
	CF_TYPE(struct switch_config),
	CF_COMMIT(switch_commit),
	CF_ITEMS {
		CF_STRING("Name", PTR_TO(struct switch_config, name)),
		CF_STRING("Serial", PTR_TO(struct switch_config, serial)),
		CF_UINT("TCPPortBase", PTR_TO(struct switch_config, tcp_port_base)),
		CF_END
	}
};

static char *config_commit(void *x UNUSED)
{
	if (clist_empty(&switch_configs))
		return "No switches defined";

	CLIST_FOR_EACH(struct switch_config *, s, switch_configs)
		if (!s->serial && clist_next(&switch_configs, &s->n))
			return "Only the last switch can be defined with no serial number";

	return NULL;
}

static struct cf_section daemon_config = {
	CF_COMMIT(config_commit),
	CF_ITEMS {
		CF_LIST("Switch", &switch_configs, &switch_config),
		CF_UINT("TCPTimeout", &tcp_timeout),
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

/*** Scheduler ***/

static struct message *sched_next_msg(struct box *box)
{
	// Round-robin on ports. Replace by a better scheduler one day.
	static int robin = 1;

	for (int i=0; i<NUM_PORTS-1; i++) {
		robin = robin + 1;
		if (robin == NUM_PORTS)
			robin = 1;
		struct port *port = &box->ports[robin];
		struct message *m;
		if (m = clist_remove_head(&port->ready_messages_qn)) {
			clist_add_tail(&box->busy_messages_qn, &m->queue_node);
			return m;
		}
	}

	return NULL;
}

static int sched_handler(struct main_hook *hook)
{
	struct box *box = hook->data;
	struct message *m;

	// Process control messages
	while (control_is_ready(box) && (m = clist_head(&box->ports[0].ready_messages_qn))) {
		clist_remove(&m->queue_node);
		clist_add_tail(&box->control_messages_qn, &m->queue_node);
		control_submit_message(m);
	}

	// Send messages to USB
	while (usb_is_ready(box)) {
		struct message *m = sched_next_msg(box);
		if (!m)
			break;
		usb_submit_message(m);
	}

	return HOOK_IDLE;
}

static void sched_init(struct box *box)
{
	box->sched_hook.handler = sched_handler;
	box->sched_hook.data = box;
	hook_add(&box->sched_hook);
}

/*** Initialization ***/

clist box_list;

static void port_init(struct box *box, int index)
{
	struct port *port = &box->ports[index];

	port->box = box;
	port->port_number = index;
	clist_init(&port->ready_messages_qn);

	port->baud_rate = 19200;
	port->parity = URS485_PARITY_EVEN;
	port->powered = 0;
	port->request_timeout = 5000;

	net_init_port(port);
}

static void box_init(struct box *box)
{
	clist_init(&box->busy_messages_qn);
	clist_init(&box->control_messages_qn);
	clist_init(&box->orphaned_messages_cn);

	for (int i=0; i<NUM_PORTS; i++)
		port_init(box, i);

	sched_init(box);
}

static void boxes_init(void)
{
	clist_init(&box_list);
	CLIST_FOR_EACH(struct switch_config *, c, switch_configs) {
		struct box *b = xmalloc_zero(sizeof(*b));
		b->cf = c;
		clist_add_tail(&box_list, &b->n);
		box_init(b);
	}
}

int main(int argc UNUSED, char **argv)
{
	cf_def_file = "config";
	cf_declare_section("Daemon", &daemon_config, 0);
	opt_parse(&options, argv+1);

	main_init();
	boxes_init();
	usb_init();

	main_loop();
}
