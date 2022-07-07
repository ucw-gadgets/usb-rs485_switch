/*
 *	An Example MODBUS Client on the BluePill
 *
 *	Pin assignment:
 *
 *		PA2	TXD2 (debugging console)
 *		PA3	RXD2 (debugging console)
 *		PB1	transceiver DE
 *		PB10	TXD3 -> transceiver D
 *		PB11	RXD3 -> transceiver R
 */

#include "util.h"
#include "modbus.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART3);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM4);

	rcc_periph_reset_pulse(RST_GPIOA);
	rcc_periph_reset_pulse(RST_GPIOB);
	rcc_periph_reset_pulse(RST_GPIOC);
	rcc_periph_reset_pulse(RST_USART2);
	rcc_periph_reset_pulse(RST_USART3);
	rcc_periph_reset_pulse(RST_TIM2);
	rcc_periph_reset_pulse(RST_TIM4);
}

static void gpio_setup(void)
{
	// PC13 = BluePill LED
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_clear(GPIOC, GPIO13);

	// Pins for MODBUS USART
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
}

static volatile u32 ms_ticks;

void sys_tick_handler(void)
{
	ms_ticks++;
}

static void tick_setup(void)
{
	systick_set_frequency(1000, 72000000);
	systick_counter_enable();
	systick_interrupt_enable();
}

static void delay_ms(uint ms)
{
	u32 start_ticks = ms_ticks;
	while (ms_ticks - start_ticks < ms)
		;
}

static void debug_setup(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable(USART2);
}

int main(void)
{
	clock_setup();
	gpio_setup();
	tick_setup();
	debug_setup();
	debug_printf("Chramst!\n");

	modbus_init();
	cm_enable_interrupts();		// FIXME: Needed?

	for (;;) {
		debug_led_toggle();
		delay_ms(50);
		modbus_loop();
	}

	return 0;
}

/*** Modbus callbacks ***/

bool modbus_check_discrete_input(u16 addr UNUSED)
{
	return false;
}

bool modbus_get_discrete_input(u16 addr UNUSED)
{
	return false;
}

bool modbus_check_coil(u16 addr UNUSED)
{
	return false;
}

bool modbus_get_coil(u16 addr UNUSED)
{
	return false;
}

void modbus_set_coil(u16 addr UNUSED, bool value UNUSED)
{
}

bool modbus_check_input_register(u16 addr UNUSED)
{
	return false;
}

u16 modbus_get_input_register(u16 addr UNUSED)
{
	return 0;
}

bool modbus_check_holding_register(u16 addr UNUSED)
{
	return (addr < 16);
}

u16 modbus_get_holding_register(u16 addr UNUSED)
{
	return 0xbeef;
}

void modbus_set_holding_register(u16 addr UNUSED, u16 value UNUSED)
{
}

void modbus_ready_hook(void)
{
}

void modbus_frame_start_hook(void)
{
}

const char * const modbus_id_strings[MODBUS_ID_MAX] = {
	[MODBUS_ID_VENDOR_NAME] = "United Computer Wizards",
	[MODBUS_ID_PRODUCT_CODE] = "42",
	[MODBUS_ID_MAJOR_MINOR_REVISION] = "1.0",
	[MODBUS_ID_VENDOR_URL] = "http://www.ucw.cz/",
	[MODBUS_ID_PRODUCT_NAME] = "Magic Gadget",
	[MODBUS_ID_USER_APP_NAME] = NULL,
};
