// Processor clock

#define CPU_CLOCK_MHZ 72

// Debugging port

// #define DEBUG_SEMIHOSTING
#define DEBUG_USART USART2
#define DEBUG_LED_BLUEPILL

// MODBUS library parameters

#define MODBUS_USART USART3
#define MODBUS_NVIC_USART_IRQ NVIC_USART3_IRQ
#define MODBUS_USART_ISR usart3_isr

#define MODBUS_TXEN_GPIO_PORT GPIOB
#define MODBUS_TXEN_GPIO_PIN GPIO1

#define MODBUS_TIMER TIM2
#define MODBUS_NVIC_TIMER_IRQ NVIC_TIM2_IRQ
#define MODBUS_TIMER_ISR tim2_isr

#define MODBUS_OUR_ADDRESS 42

#define MODBUS_BAUD_RATE 19200

#define MODBUS_DEBUG
