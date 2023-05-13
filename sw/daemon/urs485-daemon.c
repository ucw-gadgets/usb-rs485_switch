/*
 *	USB-RS485 Switch Daemon
 *
 *	(c) 2022--2023 Martin Mares <mj@ucw.cz>
 */

#include "daemon.h"

#include <fcntl.h>
#include <ucw/conf.h>
#include <ucw/fastbuf.h>
#include <ucw/log.h>
#include <ucw/opt.h>
#include <ucw/stkstring.h>
#include <unistd.h>

/*** Configuration ***/

clist switch_configs;

char *log_stream;
uint tcp_timeout = 60;
uint log_connections;
uint max_queued_messages;
char *persistent_dir;

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
		CF_STRING("LogStream", &log_stream),
		CF_UINT("TCPTimeout", &tcp_timeout),
		CF_UINT("LogConnections", &log_connections),
		CF_UINT("MaxQueued", &max_queued_messages),
		CF_STRING("PersistentDir", &persistent_dir),
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
			clist_remove(&m->client_node);
			clist_add_tail(&m->client->busy_messages_cn, &m->client_node);
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

/*** Persistent settings ***/

static void port_set_description(struct port *port, const char *desc)
{
	uint len = strlen(desc);
	len = MIN(len, PORT_DESCRIPTION_SIZE);
	memcpy(port->description, desc, len);
	memset(port->description + len, ' ', PORT_DESCRIPTION_SIZE - len);
}

static void persist_handler(struct main_timer *tm)
{
	struct box *box = tm->data;
	timer_del(tm);

	msg(L_DEBUG, "Switch %s: Saving persistent settings", box->cf->name);

	const char *filename = stk_printf("%s/%s", persistent_dir, box->cf->name);
	const char *tmpname = stk_printf("%s.new", filename);
	struct fastbuf *fb = bopen_try(tmpname, O_WRONLY | O_CREAT | O_TRUNC, 4096);
	bputsn(fb, "# baud parity powered timeout");
	bputsn(fb, "# >description");

	for (int i=1; i<NUM_PORTS; i++) {
		struct port *port = &box->ports[i];
		bprintf(fb, "%d %d %d %d\n",
			port->baud_rate,
			port->parity,
			port->powered,
			port->request_timeout
			);
		bprintf(fb, ">%.*s\n", PORT_DESCRIPTION_SIZE, port->description);
	}

	bclose(fb);

	if (rename(tmpname, filename) < 0)
		die("Cannot rename %s to %s: %m", tmpname, filename);
}

static void persist_load(struct box *box)
{
	if (!persistent_dir)
		return;

	box->persist_timer.handler = persist_handler;
	box->persist_timer.data = box;

	const char *filename = stk_printf("%s/%s", persistent_dir, box->cf->name);
	struct fastbuf *fb = bopen_try(filename, O_RDONLY, 4096);
	if (!fb) {
		msg(L_INFO, "Persistent settings %s not found", filename);
		return;
	}

	msg(L_INFO, "Switch %s: Loading persistent settings", box->cf->name);

	char line[256];
	int i = 1;
	int lino = 0;

	while (bgets(fb, line, sizeof(line))) {
		lino++;
		if (!line[0] || line[0] == '#')
			continue;

		if (line[0] == '>') {
			ASSERT(i > 1);
			struct port *port = &box->ports[i - 1];
			port_set_description(port, line + 1);
			continue;
		}

		int baud, parity, powered, timeout;
		if (sscanf(line, "%d%d%d%d", &baud, &parity, &powered, &timeout) != 4)
			die("%s:%d: Parse error", filename, lino);
		if (i >= NUM_PORTS)
			die("%s:%d: Too many ports", filename, lino);

		struct port *port = &box->ports[i];
		port->baud_rate = baud;
		port->parity = parity;
		port->powered = powered;
		port->request_timeout = timeout;
		i++;
	}

	if (i != NUM_PORTS)
		die("%s: Too few ports", filename);

	bclose(fb);
}

void persist_schedule_write(struct box *box)
{
	if (!persistent_dir)
		return;

	if (!timer_is_active(&box->persist_timer))
		timer_add_rel(&box->persist_timer, 1000);
}

/*** Logging ***/

uint log_type_client;
uint log_type_usb;

static void logging_setup(void)
{
	log_type_client = log_register_type("client");
	log_type_usb = log_register_type("usb");
}

static void logging_init(void)
{
	if (log_stream)
		log_configured(log_stream);
}

/*** Initialization ***/

clist box_list;

static void port_init(struct box *box, int index)
{
	struct port *port = &box->ports[index];

	port->box = box;
	port->port_number = index;
	port->phys_number = 8 - index;
	clist_init(&port->ready_messages_qn);

	port->baud_rate = 19200;
	port->parity = URS485_PARITY_EVEN;
	port->powered = 0;
	port->request_timeout = 5000;

	if (index > 0) {
		char desc[PORT_DESCRIPTION_SIZE + 1];
		snprintf(desc, sizeof(desc), "port%d", index);
		port_set_description(port, desc);
	} else {
		port_set_description(port, "ctrl");
	}

	net_init_port(port);
}

static void box_init(struct box *box)
{
	clist_init(&box->busy_messages_qn);
	clist_init(&box->control_messages_qn);
	clist_init(&box->orphaned_messages_cn);

	for (int i=0; i<NUM_PORTS; i++)
		port_init(box, i);

	persist_load(box);

	sched_init(box);

	msg(L_INFO, "Switch %s: Listening on TCP ports %d-%d", box->cf->name,
	    box->cf->tcp_port_base, box->cf->tcp_port_base + NUM_PORTS - 1);
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
	logging_setup();

	cf_def_file = "/etc/urs485/config";
	cf_declare_section("Daemon", &daemon_config, 0);
	opt_parse(&options, argv+1);

	logging_init();
	main_init();
	boxes_init();
	usb_init();

	main_loop();
}
