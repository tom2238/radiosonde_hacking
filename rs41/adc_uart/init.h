#ifndef STM32_INIT_C
#define STM32_INIT_C

// Vaisala RS41 leds
#define LED_GREEN_GPIO GPIOB
#define LED_GREEN_PIN GPIO7
#define LED_GREEN_RCC RCC_GPIOB
#define LED_RED_GPIO GPIOB
#define LED_RED_PIN GPIO8
#define LED_RED_RCC RCC_GPIOB

// Vaisala RS41 XDATA UART
#define XDATA_USART USART3
#define XDATA_USART_GPIO GPIOB
#define XDATA_USART_RCC_GPIO RCC_GPIOB
#define XDATA_USART_RCC RCC_USART3

// Vaisala RS41 battery voltage
#define VBAT_MON_GPIO GPIOA
#define VBAT_MON_PIN GPIO5
#define VBAT_MON_RCC RCC_GPIOA
#define VBAT_MON_ADC_CHANNEL ADC_CHANNEL5

void gpio_setup(void);
void adc_setup(void);
void usart_setup(void);
void clock_setup(void);
void systick_setup(void);
uint64_t millis(void);
void delay(uint64_t duration);

#endif // STM32_INIT_C
