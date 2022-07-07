PC=pkg-config
UCW_CFLAGS := $(shell $(PC) --cflags libucw)
UCW_LIBS := $(shell $(PC) --libs libucw)
USB_CFLAGS := $(shell $(PC) --cflags libusb-1.0)
USB_LIBS := $(shell $(PC) --libs libusb-1.0)

CFLAGS=-O2 -Wall -Wextra -Wno-sign-compare -Wno-parentheses -Wstrict-prototypes -Wmissing-prototypes $(UCW_CFLAGS) $(USB_CFLAGS)
LDLIBS=$(UCW_LIBS) $(USB_LIBS)

all: urs-test

urs-test.o: urs-test.c ../firmware/interface.h

clean:
	rm -f *.o urs-test

.PHONY: all install clean
