PC=pkg-config
UCW_CFLAGS := $(shell $(PC) --cflags libucw)
UCW_LIBS := $(shell $(PC) --libs libucw)
USB_CFLAGS := $(shell $(PC) --cflags libusb-1.0)
USB_LIBS := $(shell $(PC) --libs libusb-1.0)

CFLAGS=-O2 -Wall -Wextra -Wno-sign-compare -Wno-parentheses -Wstrict-prototypes -Wmissing-prototypes $(UCW_CFLAGS) $(USB_CFLAGS)
LDLIBS=$(UCW_LIBS) $(USB_LIBS)

all: urs485-daemon

urs485-daemon: urs485-daemon.o usb.o

urs485-daemon.o: urs485-daemon.c usb.h ../firmware/interface.h
usb.o: usb.c usb.h

install: urs485-daemon
	install urs485-daemon /usr/local/sbin/

clean:
	rm -f *.o urs485-daemon

.PHONY: all install clean
