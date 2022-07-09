/*
 *	LibUSB over LibUCW Mainloop
 *
 *	(c) 2014--2022 Martin Mares <mj@ucw.cz>
 */

#include <libusb.h>

extern libusb_context *usb_ctx;

void usb_init_mainloop(void);
