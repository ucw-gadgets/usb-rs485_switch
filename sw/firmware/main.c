/*
 *	USB-RS485 Switch -- Firmware
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "firmware.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>

#include <string.h>

/*** Message queues ***/

static struct message_node message_buf[MAX_IN_FLIGHT];

struct message_queue idle_queue;
struct message_queue done_queue;

void queue_put(struct message_queue *q, struct message_node *n)
{
	if (q->last)
		q->last->next = n;
	else
		q->first = n;
	q->last = n;
	n->next = NULL;

	// We will not wait for an interrupt before the next iteration of the
	// main loop, so that other modules get a chance to react quickly.
	retry_loop();
}

struct message_node *queue_get(struct message_queue *q)
{
	struct message_node *n = q->first;
	if (n) {
		q->first = n->next;
		if (!q->first)
			q->last = NULL;
	}
	return n;
}

static void queues_init(void)
{
	for (uint i=0; i < MAX_IN_FLIGHT; i++)
		queue_put(&idle_queue, &message_buf[i]);
}

/*** Global status ***/

struct urs485_port_params port_params[8];
struct urs485_port_status port_status[8];
struct urs485_power_status power_status;

const struct urs485_config global_config = {
	.max_in_flight = MAX_IN_FLIGHT,
};

static void params_init(void)
{
	for (int i=0; i<8; i++) {
		struct urs485_port_params *pp = &port_params[i];
		pp->baud_rate = 9600;
		pp->parity = URS485_PARITY_EVEN;
		pp->request_timeout = 1000;
	}
}

/*** Hardware init ***/

static void clock_init(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART1);	// channel 0
	rcc_periph_clock_enable(RCC_USART2);	// debugging
	rcc_periph_clock_enable(RCC_USART3);	// channel 1
	rcc_periph_clock_enable(RCC_SPI2);
	rcc_periph_clock_enable(RCC_TIM2);	// channel 0
	rcc_periph_clock_enable(RCC_TIM3);	// channel 1
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_AFIO);

	rcc_periph_reset_pulse(RST_GPIOA);
	rcc_periph_reset_pulse(RST_GPIOB);
	rcc_periph_reset_pulse(RST_GPIOC);
	rcc_periph_reset_pulse(RST_USART1);
	rcc_periph_reset_pulse(RST_USART2);
	rcc_periph_reset_pulse(RST_USART3);
	rcc_periph_reset_pulse(RST_SPI2);
	rcc_periph_reset_pulse(RST_TIM2);
	rcc_periph_reset_pulse(RST_TIM3);
	rcc_periph_reset_pulse(RST_ADC1);
	rcc_periph_reset_pulse(RST_AFIO);
}

static void gpio_init(void)
{
	// PA0 = VCC sense -> ADC12_IN0
	// PA1 = 5V sense -> ADC12_IN1
	// PA2 = TXD2 (debug)
	// PA3 = RXD2
	// PA4 = SPI1_SS*  (jumpers)
	// PA5 = SPI1_SCK
	// PA6 = SPI1_MISO
	// PA7 = SPI1_MOSI
	// PA8 = SPI2_OE*
	// PA9 = TXD1
	// PA10 = RXD1
	// PA11 = USB_DN
	// PA12 = USB_DP
	// PA13 + PA14 = programming interface
	// PA15 = GPIO3

	// PB0 = GPIO0 = ADuM4160 PIN (USB upstream enable)
	// PB1 = I_SENSE -> ADC12_IN9
	// PB2 = unused
	// PB3 = I_SEN_A
	// PB4 = I_SEN_B
	// PB5 = I_SEN_C
	// PB6 = I2C1_SCL
	// PB7 = I2C1_SDA
	// PB8 = GPIO1
	// PB9 = GPIO2
	// PB10 = TXD3
	// PB11 = RXD3
	// PB12 = SPI2_STROBE
	// PB13 = SPI2_SCK
	// PB14 = SPI2_MISO
	// PB15 = SPI2_MOSI

	// PC13 = debugging LED *
	// PC14 = unused
	// PC15 = unused

	// Switch JTAG off to free up pins
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);
}

static void debug_init(void)
{
	// PC13 = debugging LED à la BluePill
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_clear(GPIOC, GPIO13);

	// USART2: Debugging console
	// PA2 = TXD2
	// PA3 = RXD2
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable(USART2);
}

/*** System ticks ***/

volatile u32 ms_ticks;

void sys_tick_handler(void)
{
	ms_ticks++;
}

static void tick_init(void)
{
	systick_set_frequency(1000, CPU_CLOCK_MHZ * 1000000);
	systick_counter_enable();
	systick_interrupt_enable();
}

void delay_ms(uint ms)
{
	u32 start_ticks = ms_ticks;
	while (ms_ticks - start_ticks < ms)
		;
}

/*** Shift registers ***/

static void reg_init(void)
{
	// Pins: PA8=OE*, PB12=STROBE, PB13=SCK, PB14=MISO, PB15=MOSI
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
}

static byte reg_state[4] = { 0x88, 0x88, 0x88, 0x88 };

void reg_send(void)
{
	for (byte i=0; i<4; i++)
		spi_send(SPI2, reg_state[3-i]);
	while (SPI_SR(SPI2) & SPI_SR_BSY)
		;
	gpio_set(GPIOB, GPIO12);	// strobe
	asm volatile ("nop; nop; nop; nop");	// just to be sure
	gpio_clear(GPIOB, GPIO12);
	gpio_clear(GPIOA, GPIO8);	// OE*
}

void reg_set_flag(uint port, uint flag)
{
	reg_state[port/2] |= (port & 1) ? flag : (flag << 4);
}

void reg_clear_flag(uint port, uint flag)
{
	reg_state[port/2] &= ~((port & 1) ? flag : (flag << 4));
}

void reg_toggle_flag(uint port, uint flag)
{
	reg_state[port/2] ^= (port & 1) ? flag : (flag << 4);
}

/*** ADC ***/

static void adc_init(void)
{
	// PA0, PA1 and PB1 are analog inputs
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

	// PB3 to PB5 control the analog multiplexer on PB1
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3 | GPIO4 | GPIO5);

	adc_power_off(ADC1);
	// ADC prescaler set by rcc_clock_setup_pll(): ADC clock = PCLK2/8 = 9 MHz
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL9, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL16, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL17, ADC_SMPR_SMP_239DOT5CYC);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_enable_temperature_sensor();
	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

/*** Main ***/

static bool main_retry;

void retry_loop(void)
{
	main_retry = true;
}

int main(void)
{
	clock_init();
	gpio_init();
	debug_init();
	params_init();
	reg_init();
	tick_init();
	adc_init();
	usb_init();
	queues_init();
	bus_init();

	debug_printf("USB-RS485 Switch (version %04x)\n", URS485_USB_VERSION);

	u32 last_blink = 0;

	for (;;) {
		main_retry = false;

		if (ms_ticks - last_blink >= 250) {
			debug_led_toggle();
			last_blink = ms_ticks;
		}

		if (usart_get_flag(USART2, USART_SR_RXNE)) {
			uint ch = usart_recv(USART2);
			debug_putc(ch);
		}

		bus_loop();
		usb_loop();

		if (!main_retry)
			wait_for_interrupt();
	}

	return 0;
}
