/*
 *	USB-RS485 Switch -- Hardware Testing
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#include "util.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/usb/usbd.h>

#include <string.h>

/*** Hardware init ***/

static void clock_init(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART3);
	rcc_periph_clock_enable(RCC_SPI2);
	// rcc_periph_clock_enable(RCC_TIM4);
	rcc_periph_clock_enable(RCC_ADC1);

	rcc_periph_reset_pulse(RST_GPIOA);
	rcc_periph_reset_pulse(RST_GPIOB);
	rcc_periph_reset_pulse(RST_GPIOC);
	rcc_periph_reset_pulse(RST_USART1);
	rcc_periph_reset_pulse(RST_USART2);
	rcc_periph_reset_pulse(RST_USART3);
	rcc_periph_reset_pulse(RST_SPI2);
	// rcc_periph_reset_pulse(RST_TIM4);
	rcc_periph_reset_pulse(RST_ADC1);
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

static volatile u32 ms_ticks;

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

static void delay_ms(uint ms)
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

static void reg_send(void)
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

enum reg_flags {
	SF_RXEN_N = 8,
	SF_TXEN = 4,
	SF_PWREN = 2,
	SF_LED = 1,
};

static void reg_set_flag(uint port, uint flag)
{
	reg_state[port/2] |= (port & 1) ? flag : (flag << 4);
}

static void reg_clear_flag(uint port, uint flag)
{
	reg_state[port/2] &= ~((port & 1) ? flag : (flag << 4));
}

static void reg_toggle_flag(uint port, uint flag)
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

static void adc_test(void)
{
	for (int i=0; i<4; i++) {
		uint j=0;
		switch (i) {
		case 0:
			j = ADC_CHANNEL0;
			break;
		case 1:
			j = ADC_CHANNEL1;
			break;
		case 2:
			j = ADC_CHANNEL16;
			break;
		case 3:
			j = ADC_CHANNEL17;
			break;
		}

		uint8_t channels[] = { j };
		adc_set_regular_sequence(ADC1, 1, channels);

		adc_start_conversion_regular(ADC1);

		while (! adc_eoc(ADC1));
		uint reg16 = adc_read_regular(ADC1);
		debug_printf("ADC %d = %d\n", i, reg16);
	}

	for (int i=0; i<8; i++) {
		gpio_clear(GPIOB, GPIO3 | GPIO4 | GPIO5);
		gpio_set(GPIOB, i << 3);
		delay_ms(5);

		uint8_t channels[] = { ADC_CHANNEL9 };
		adc_set_regular_sequence(ADC1, 1, channels);

		adc_start_conversion_regular(ADC1);

		while (! adc_eoc(ADC1));
		uint reg16 = adc_read_regular(ADC1);
		debug_printf("SENSE %d = %d\n", i, reg16);
	}
}

/*** USB ***/

#define USB_RS485_USB_VENDOR 0x4242
#define USB_RS485_USB_PRODUCT 0x000b
#define USB_RS485_USB_VERSION 1

static usbd_device *usbd_dev;

enum usb_string {
	STR_MANUFACTURER = 1,
	STR_PRODUCT,
	STR_SERIAL,
};

static char usb_serial_number[13];

static const char *usb_strings[] = {
	"United Computer Wizards",
	"USB-RS485 Switch",
	usb_serial_number,
};

static const struct usb_device_descriptor device = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFF,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = USB_RS485_USB_VENDOR,
	.idProduct = USB_RS485_USB_PRODUCT,
	.bcdDevice = USB_RS485_USB_VERSION,
	.iManufacturer = STR_MANUFACTURER,
	.iProduct = STR_PRODUCT,
	.iSerialNumber = STR_SERIAL,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor endpoints[] = {{
	// Bulk end-point for sending LED values
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
	.endpoint = endpoints,
};

static const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x0100,
};

static const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 0,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}, {
	.num_altsetting = 1,
	.altsetting = &dfu_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 100,	// multiplied by 2 mA
	.interface = ifaces,
};

static byte usb_configured;
static uint8_t usbd_control_buffer[64];

static void dfu_detach_complete(usbd_device *dev UNUSED, struct usb_setup_data *req UNUSED)
{
	// Reset to bootloader, which implements the rest of DFU
	debug_printf("Switching to DFU\n");
	debug_flush();
	scb_reset_core();
}

static enum usbd_request_return_codes dfu_control_cb(usbd_device *dev UNUSED,
	struct usb_setup_data *req,
	uint8_t **buf UNUSED,
	uint16_t *len UNUSED,
	void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	if (req->bmRequestType != 0x21 || req->bRequest != DFU_DETACH)
		return USBD_REQ_NOTSUPP;

	*complete = dfu_detach_complete;
	return USBD_REQ_HANDLED;
}

static byte usb_rx_buf[64];

static void ep01_cb(usbd_device *dev, uint8_t ep UNUSED)
{
	// We received a frame from the USB host
	uint len = usbd_ep_read_packet(dev, 0x01, usb_rx_buf, sizeof(usb_rx_buf));
	debug_printf("USB: Host sent %u bytes\n", len);
}

static void set_config_cb(usbd_device *dev, uint16_t wValue UNUSED)
{
	usbd_register_control_callback(
		dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		dfu_control_cb);
	usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, ep01_cb);
	usb_configured = 1;
}

static void reset_cb(void)
{
	debug_printf("USB: Reset\n");
	usb_configured = 0;
}

static volatile bool usb_event_pending;

void usb_lp_can_rx0_isr(void)
{
	/*
	 *  We handle USB in the main loop to avoid race conditions between
	 *  USB interrupts and other code. However, we need an interrupt to
	 *  up the main loop from sleep.
	 *
	 *  We set up only the low-priority ISR, because high-priority ISR handles
	 *  only double-buffered bulk transfers and isochronous transfers.
	 */
	nvic_disable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	usb_event_pending = 1;
}

static void usb_init(void)
{
	// Simulate USB disconnect
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
	gpio_clear(GPIOB, GPIO0);
	delay_ms(100);
	gpio_set(GPIOB, GPIO0);

	usbd_dev = usbd_init(
		&st_usbfs_v1_usb_driver,
		&device,
		&config,
		usb_strings,
		ARRAY_SIZE(usb_strings),
		usbd_control_buffer,
		sizeof(usbd_control_buffer)
	);
	usbd_register_reset_callback(usbd_dev, reset_cb);
	usbd_register_set_config_callback(usbd_dev, set_config_cb);
	usb_event_pending = 1;
}

/*** Main ***/

int main(void)
{
	clock_init();
	gpio_init();
	debug_init();
	reg_init();
	tick_init();
	adc_init();
	usb_init();

	debug_printf("Lisak je lisak...\n");

	byte t = 0;
	u32 last_blink = 0;

	for (;;) {
		if (ms_ticks - last_blink >= 250) {
			debug_led_toggle();
			reg_clear_flag(t, SF_LED);
			t = (t+1) & 7;
			reg_set_flag(t, SF_LED);
			reg_send();
			last_blink = ms_ticks;
		}

		if (usart_get_flag(USART2, USART_SR_RXNE)) {
			uint ch = usart_recv(USART2);
			debug_putc(ch);
			if (ch >= '0' && ch <= '7') {
				reg_toggle_flag(ch - '0', SF_PWREN);
				reg_send();
			} else if (ch == 'a') {
				adc_test();
			}
		}

		if (usb_event_pending) {
			usbd_poll(usbd_dev);
			usb_event_pending = 0;
			nvic_clear_pending_irq(NVIC_USB_LP_CAN_RX0_IRQ);
			nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
		}

		wait_for_interrupt();
	}

	return 0;
}
