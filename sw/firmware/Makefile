STM32LIB=../../../stm32lib
OPENCM3_DIR=/home/mj/stm/libopencm3
BINARY=firmware
OBJS=main.o bus.o usb.o
LIB_OBJS=util-debug.o

WITH_BOOT_LOADER=1
WITH_DFU_FLASH=1
DFU_ARGS=-d 4242:000b

include $(STM32LIB)/mk/bluepill.mk

main.o: firmware.h interface.h
bus.o: firmware.h interface.h
usb.o: firmware.h interface.h
