/*
 *	USB-RS485 Switch Daemon -- Client Connections
 *
 *	(c) 2022 Martin Mares <mj@ucw.cz>
 */

#define LOCAL_DEBUG

#include "daemon.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <ucw/unaligned.h>
#include <unistd.h>

#define CLIENT_MSG(client, level, fmt, ...) msg(level, "Client %d: " fmt, client->id, ##__VA_ARGS__)
#define CLIENT_DBG(client, fmt, ...) DBG("Client %d: " fmt, client->id, ##__VA_ARGS__)

struct tcp_modbus_header {
	// All fields are big-endian
	u16 transaction_id;
	u16 protocol_id;		// Always 0
	u16 length;
} __attribute__((packed));

static struct message *msg_new(struct client *client)
{
	struct message *m = xmalloc_zero(sizeof(*m));
	m->box = client->box;
	m->client = client;
	m->port = client->port;
	return m;
}

void msg_free(struct message *m)
{
	clist_remove(&m->queue_node);
	clist_remove(&m->client_node);
	if (m->ctrl)
		xfree(m->ctrl);
	xfree(m);
}

void msg_send_reply(struct message *m)
{
	if (m->request[0] == 0) {
		CLIENT_DBG(m->client, "Not replying to broadcast #%04x", m->client_transaction_id);
		msg_free(m);
		return;
	}

	if (!m->client) {
		DBG("Dropping reply to an orphaned message #%04x", m->client_transaction_id);
		msg_free(m);
		return;
	}

	struct main_rec_io *rio = &m->client->rio;
	CLIENT_DBG(m->client, "Sending frame #%04x of %u bytes", m->client_transaction_id, m->reply_size);

	struct tcp_modbus_header h;
	put_u16_be(&h.transaction_id, m->client_transaction_id);
	h.protocol_id = 0;
	put_u16_be(&h.length, m->reply_size);
	rec_io_write(rio, &h, sizeof(h));

	rec_io_write(rio, m->reply, m->reply_size);

	msg_free(m);
}

void msg_send_error_reply(struct message *m, enum modbus_error err)
{
	CLIENT_DBG(m->client, "Message #%04x failed with error %02x", m->client_transaction_id, err);
	m->reply[0] = m->request[0];
	m->reply[1] = m->request[1] | 0x80;
	m->reply[2] = err;
	m->reply_size = 3;
	msg_send_reply(m);
}

static struct client *client_new(struct port *port, int id)
{
	struct client *c = xmalloc_zero(sizeof(*c));
	c->id = id;
	c->box = port->box;
	c->port = port;
	clist_init(&c->rx_messages_cn);
	clist_init(&c->busy_messages_cn);
	return c;
}

static void client_free(struct client *client)
{
	rec_io_del(&client->rio);
	close(client->rio.file.fd);

	// Received, but unprocessed messages can be removed
	CLIST_FOR_EACH(cnode *, mn, client->rx_messages_cn) {
		struct message *m = SKIP_BACK(struct message, client_node, mn);
		CLIENT_DBG(client, "Dropping message #%04x", m->client_transaction_id);
		msg_free(m);
	}

	// Messages which are being processed are turned into orphans
	cnode *mn;
	while (mn = clist_head(&client->busy_messages_cn)) {
		struct message *m = SKIP_BACK(struct message, client_node, mn);
		CLIENT_DBG(client, "Orphaning message #%04x", m->client_transaction_id);
		clist_remove(mn);
		clist_add_tail(&client->box->orphaned_messages_cn, mn);
		m->client = NULL;
	}

	CLIENT_DBG(client, "Destroyed");
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
		CLIENT_MSG(client, L_ERROR_R, "Received invalid protocol ID #%04x", protocol_id);
		client_free(client);
		return ~0U;
	}
	uint len = get_u16_be(&h->length);
	if (len < 2) {
		CLIENT_MSG(client, L_ERROR_R, "Received undersized frame of %u bytes", len);
		client_free(client);
		return ~0U;
	}
	if (len > 2 + MODBUS_MAX_DATA_SIZE) {
		CLIENT_MSG(client, L_ERROR_R, "Received oversized frame of %u bytes", len);
		client_free(client);
		return ~0U;
	}
	if (rio->read_avail < sizeof(struct tcp_modbus_header) + len)
		return 0;

	struct message *m = msg_new(client);
	m->client_transaction_id = get_u16_be(&h->transaction_id);
	m->request_size = 2 + len;
	memcpy(m->request, rio->read_buf + sizeof(struct tcp_modbus_header), len);

	clist_add_tail(&client->port->ready_messages_qn, &m->queue_node);
	clist_add_tail(&client->rx_messages_cn, &m->client_node);

	CLIENT_DBG(client, "Received frame #%04x of %u bytes for port %d", m->client_transaction_id, m->request_size, m->port->port_number);

	rec_io_set_timeout(rio, tcp_timeout * 1000);
	return sizeof(struct tcp_modbus_header) + len;
}

static int sk_notify_handler(struct main_rec_io *rio, int status)
{
	struct client *client = rio->data;

	if (status < 0) {
		switch (status) {
			case RIO_ERR_READ:
				CLIENT_MSG(client, L_ERROR_R, "Read error: %m");
				break;
			case RIO_ERR_WRITE:
				CLIENT_MSG(client, L_ERROR_R, "Write error: %m");
				break;
			default:
				CLIENT_MSG(client, L_ERROR_R, "Unknown error");
		}
		client_free(client);
		return HOOK_IDLE;
	} else if (status == RIO_EVENT_EOF) {
		CLIENT_MSG(client, L_INFO_R, "Closed connection");
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
	socklen_t peer_addr_len = sizeof(peer_addr);
	int sk = accept(fi->fd, &peer_addr, &peer_addr_len);
	if (sk < 0) {
		if (errno == EAGAIN)
			return HOOK_IDLE;
		msg(L_ERROR, "Error accepting connection: %m");
		return HOOK_RETRY;
	}

	char name_buf[INET6_ADDRSTRLEN];
	const char *peer_name = inet_ntop(AF_INET6, &peer_addr.sin6_addr, name_buf, sizeof(name_buf));
	ASSERT(peer_name);

	struct client *client = client_new(port, sk);
	struct main_rec_io *rio = &client->rio;
	rio->read_handler = sk_read_handler;
	rio->notify_handler = sk_notify_handler;
	rio->data = client;
	rio->write_throttle_read = 16384;
	rec_io_add(rio, sk);
	rec_io_start_read(rio);
	rec_io_set_timeout(rio, tcp_timeout * 1000);

	CLIENT_MSG(client, L_INFO_R, "New connection from %s for port %s/%d", peer_name, port->box->cf->name, port->port_number);

	return HOOK_RETRY;
}

void net_init_port(struct port *port)
{
	int sk = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
	if (sk < 0)
		die("Cannot create TCPv6 socket: %m");

	int one = 1;
	if (setsockopt(sk, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0)
		die("Cannot set SO_REUSEADDR on socket: %m");

	uint tcp_port = port->box->cf->tcp_port_base + port->port_number;
	struct sockaddr_in6 sin = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(tcp_port),
		.sin6_addr = IN6ADDR_ANY_INIT,
	};
	if (bind(sk, &sin, sizeof(sin)) < 0)
		die("Cannot bind on port %d: %m", tcp_port);

	if (listen(sk, 64) < 0)
		die("Cannost listen in port 4300: %m");

	port->listen_file.fd = sk;
	port->listen_file.read_handler = listen_handler;
	port->listen_file.data = &port[0];
	file_add(&port->listen_file);
}
