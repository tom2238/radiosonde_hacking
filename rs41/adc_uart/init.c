// Global LibOpenCM3 libraries
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/adc.h>
// Another libraries
#include <stdint.h>
#include "init.h"

// Storage for our monotonic system clock.
// Note that it needs to be volatile since we're modifying it from an interrupt.
static volatile uint64_t _millis = 0;

void gpio_setup(void) {
    /* Enable the correct clock. */
    /* Enable GPIO RCC clock. */
    rcc_periph_clock_enable(LED_GREEN_RCC);
    rcc_periph_clock_enable(LED_RED_RCC);

    /* Set LED_PIN to 'output push-pull'. */
    gpio_set_mode(LED_GREEN_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_GREEN_PIN);
    gpio_set_mode(LED_RED_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_RED_PIN);
}

void adc_setup(void) {
    // ADC0 setup clock
    rcc_periph_clock_enable(VBAT_MON_RCC);
    // Analog input pin, PA5, battery voltage measure
    gpio_set_mode(VBAT_MON_GPIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, VBAT_MON_PIN);
    // Initialize ADC
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    adc_power_off(ADC1);
    rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    //rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6); // Set 12 MHz, doing in clock_setup()
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND); // Independent mode
    adc_disable_scan_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_239DOT5CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL_VREF, ADC_SMPR_SMP_239DOT5CYC);
    adc_enable_temperature_sensor();
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate_async(ADC1);
    while (adc_is_calibrating(ADC1));
}

void usart_setup(void) {
    /* Enable clocks for GPIO port B (for GPIO_USART3_TX) and USART3. */
    rcc_periph_clock_enable(XDATA_USART_RCC_GPIO);
    rcc_periph_clock_enable(XDATA_USART_RCC);
    rcc_periph_clock_enable(RCC_AFIO);

    /* Setup GPIO pin GPIO_USART3_TX/GPIO9 on GPIO port B for transmit. */
    gpio_set_mode(XDATA_USART_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

    /* Setup UART parameters. */
    usart_set_baudrate(XDATA_USART, 9600);
    usart_set_databits(XDATA_USART, 8);
    usart_set_stopbits(XDATA_USART, USART_STOPBITS_1);
    usart_set_mode(XDATA_USART, USART_MODE_TX);
    usart_set_parity(XDATA_USART, USART_PARITY_NONE);
    usart_set_flow_control(XDATA_USART, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(XDATA_USART);
}

void clock_setup(void) {
    /* hse24, pll to 24 */
    /* 1/24 MHz = perioda 41.6666 ns */
    const struct rcc_clock_scale clock = {
        .pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL2,    // Nasob zdroj 2 krat
        .pll_source = RCC_CFGR_PLLSRC_HSE_CLK,      // Zdrojem je HSE 24 MHz (krystal)
        .hpre = RCC_CFGR_HPRE_DIV2,                 // Hlavni delicka za PLL 48/2 = 24 MHz
        .adcpre = RCC_CFGR_ADCPRE_DIV2,             // Delicka k ADC, 24/2 = 12 MHz
        .ppre1 = RCC_CFGR_PPRE_NODIV,               // Delicka APB1 (max 24 MHz), 24/1 = 24 MHz
        .ppre2 = RCC_CFGR_PPRE_NODIV,               // Delicka APB2 (max 24 MHz), 24/1 = 24 MHz
        .flash_waitstates = 0x00,                   // 0WS from 0-24MHz, 1WS from 24-48MHz, 2WS from 48-72MHz
        .prediv1 = RCC_CFGR2_PREDIV_NODIV,          // Delicka pred PLL od HSE krystalu, 24/1 = 24 MHz
        .ahb_frequency  = 24000000,  // Vysledne frekvence za AHB
        .apb1_frequency = 24000000,  // Vysledne frekvence za APB1
        .apb2_frequency = 24000000,  // Vysledne frekvence za APB2
    };

    rcc_clock_setup_pll(&clock);
}

void systick_setup(void) {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 - 1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}

// Get the current value of the millis counter
uint64_t millis(void) {
    return _millis;
}

// This is our interrupt handler for the systick reload interrupt.
// The full list of interrupt services routines that can be implemented is
// listed in libopencm3/include/libopencm3/stm32/f0/nvic.h
void sys_tick_handler(void) {
    // Increment our monotonic clock
    _millis++;
}

// Delay a given number of milliseconds in a blocking manner
void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until);
}
