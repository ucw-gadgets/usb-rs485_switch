/*
 *	USB-RS485 Switch -- Custom Bootloader Init
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "util.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

void custom_hw_init(void)
{
	// Reset all shift registers connected via SPI
	// Pins: PA8=OE*, PB12=STROBE, PB13=SCK, PB14=MISO, PB15=MOSI
	rcc_periph_clock_enable(RCC_SPI2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 | GPIO15);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);

	gpio_set(GPIOA, GPIO8);
	gpio_clear(GPIOB, GPIO12);

	spi_reset(SPI2);

	/*
	 * Set up SPI in Master mode with:
	 *
	 *	- baud rate: 1/64 of peripheral clock frequency
	 *	- clock polarity: idle high
	 *	- clock phase: data valid on 2nd clock pulse
	 *	- data frame format: 8-bit, MSB first
	 */
	spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

	// NSS will be managed by software and always held high
	spi_enable_software_slave_management(SPI2);
	spi_set_nss_high(SPI2);

	spi_enable(SPI2);

	for (byte i=0; i<4; i++)
		spi_send(SPI2, 0x88);
	while (SPI_SR(SPI2) & SPI_SR_BSY)
		;

	gpio_set(GPIOB, GPIO12);	// strobe
	asm volatile ("nop; nop; nop; nop");	// just to be sure
	gpio_clear(GPIOB, GPIO12);
	gpio_clear(GPIOA, GPIO8);	// OE*

	// Disable USB Isolator for 100 ms and then turn it on
        gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
        gpio_clear(GPIOB, GPIO0);
	for (uint i=0; i<100; i++) {
		while (!systick_get_countflag())
			;
	}
        gpio_set(GPIOB, GPIO0);
}
