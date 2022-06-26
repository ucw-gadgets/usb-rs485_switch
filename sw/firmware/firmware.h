/*
 *	USB-RS485 Switch -- Firmware
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "util.h"
#include "interface.h"

/*** Utilities (main.c) ***/

void delay_ms(uint ms);

/*** Message queues (main.c) ***/

#define MAX_IN_FLIGHT 2		// FIXME

struct message_node {
	struct message_node *next;
	struct urs485_message msg;
};

struct message_queue {
	struct message_node *first, *last;
};

static inline bool queue_is_empty(struct message_queue *q)
{
	return !q->first;
}

void queue_put(struct message_queue *q, struct message_node *n);
struct message_node *queue_get(struct message_queue *q);

extern struct message_queue idle_queue;		// free message structures
extern struct message_queue done_queue;		// to pass to the host

/*** Global status (main.c) ***/

extern struct urs485_port_params port_params[8];
extern struct urs485_port_status port_status[8];
extern struct urs485_power_status power_status;
extern const struct urs485_config global_config;

/*** Shift registers (main.c) ***/

enum reg_flags {
	SF_RXEN_N = 8,
	SF_TXEN = 4,
	SF_PWREN = 2,
	SF_LED = 1,
};

void reg_send(void);
void reg_set_flag(uint port, uint flag);
void reg_clear_flag(uint port, uint flag);
void reg_toggle_flag(uint port, uint flag);

/*** MODBUS (bus.c) ***/

#define MSG_HEADER_SIZE offsetof(struct urs485_message, frame)

void bus_init(void);
void bus_loop(void);

bool set_port_params(uint port, struct urs485_port_params *par);

void got_msg_from_usb(struct message_node *m);

/*** USB (usb.c) ***/

void usb_init(void);
void usb_loop(void);
