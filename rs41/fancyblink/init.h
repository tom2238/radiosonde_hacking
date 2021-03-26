#ifndef STM32_INIT_C
#define STM32_INIT_C

// Vaisala RS41 leds
#define LED_GREEN_GPIO GPIOB
#define LED_GREEN_PIN GPIO7
#define LED_GREEN_RCC RCC_GPIOB
#define LED_RED_GPIO GPIOB
#define LED_RED_PIN GPIO8
#define LED_RED_RCC RCC_GPIOB

void gpio_setup(void);

void clock_setup(void);

void systick_setup(void);

uint64_t millis(void);

void delay(uint64_t duration);

#endif // STM32_INIT_C
