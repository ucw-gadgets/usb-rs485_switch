/*
 *	USB-RS485 Switch Bootloader -- Configuration
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

// Processor clock

#define CPU_CLOCK_MHZ 48

// Debugging port

#define DEBUG_USART USART2
#define DEBUG_LED_BLUEPILL

// Bootloader settings

#undef BOOTLOADER_DEBUG
#define BOOTLOADER_APP_START 0x08002000
#define BOOTLOADER_MFG_ID 0x4242
#define BOOTLOADER_PROD_ID 0x000c
#define BOOTLOADER_PROD_VERSION 0x0100
#define BOOTLOADER_MFG_NAME "United Computer Wizards"
#define BOOTLOADER_PROD_NAME "USB-RS485 Switch"

#define BOOTLOADER_CUSTOM_HW_INIT
void custom_hw_init(void);
