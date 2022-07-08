/*
 *	LibUSB over LibUCW Mainloop
 *
 *	(c) 2014--2022 Martin Mares <mj@ucw.cz>
 */

#undef LOCAL_DEBUG

#include <ucw/lib.h>
#include <ucw/clists.h>
#include <ucw/gary.h>
#include <ucw/mainloop.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/poll.h>

#include "usb.h"

libusb_context *usb_ctx;

static struct main_file **usb_fds;

static int usb_fd_ready(struct main_file *f UNUSED)
{
	DBG("USB: Handling events (ready on fd %d)", f->fd);
	struct timeval tv = { 0, 0 };
	int comp = 0;
	int err = libusb_handle_events_timeout_completed(usb_ctx, &tv, &comp);
	if (err < 0)
		msg(L_ERROR, "libusb_handle_events: error %d", err);
	return HOOK_IDLE;
}

static void usb_added_fd(int fd, short events, void *user_data UNUSED)
{
	if (fd >= (int) GARY_SIZE(usb_fds))
		GARY_RESIZE(usb_fds, fd + 1);

	struct main_file *f = usb_fds[fd];
	if (!f) {
		f = xmalloc_zero(sizeof(*f));
		usb_fds[fd] = f;
	} else if (file_is_active(f)) {
		DBG("USB: Releasing fd %d", fd);
		file_del(f);
	}

	DBG("USB: Adding fd %d with event mask %u", fd, events);
	f->fd = fd;
	f->read_handler = (events & POLLIN) ? usb_fd_ready : NULL;
	f->write_handler = (events & POLLOUT) ? usb_fd_ready : NULL;
	file_add(f);
}

static void usb_removed_fd(int fd, void *user_data UNUSED)
{
	DBG("USB: Releasing fd %d", fd);
	ASSERT(fd < (int) GARY_SIZE(usb_fds));
	struct main_file *f = usb_fds[fd];
	ASSERT(f);
	ASSERT(file_is_active(f));
	file_del(f);
}

void usb_init(void)
{
	int err;

	// Initialize libusb
	if ((err = libusb_init(&usb_ctx)) < 0)
		die("libusb_init failed: error %d", err);

	// Connect libusb to UCW mainloop

	if (!libusb_pollfds_handle_timeouts(usb_ctx))
		die("Unsupported version of libusb, please fix me");

	GARY_INIT_ZERO(usb_fds, 0);
	libusb_set_pollfd_notifiers(usb_ctx, usb_added_fd, usb_removed_fd, NULL);

	const struct libusb_pollfd **fds = libusb_get_pollfds(usb_ctx);
	ASSERT(fds);
	for (int i=0; fds[i]; i++)
		usb_added_fd(fds[i]->fd, fds[i]->events, NULL);
	free(fds);
}
