/*
 *	USB-RS485 Switch -- Configuration
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

// Processor clock

#define CPU_CLOCK_MHZ 72

// Debugging port

#define DEBUG_USART USART2
#define DEBUG_LED_BLUEPILL

// Debug message switches

#undef DEBUG_USB
#undef DEBUG_MODBUS
