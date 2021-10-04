/*
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

// Global LibOpenCM3 libraries
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/timer.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "utils.h"

void gpio_setup(void) {
    /* Enable the correct clock. */
    /* Enable GPIO RCC clock. */
    rcc_periph_clock_enable(LED_GREEN_RCC);
    rcc_periph_clock_enable(LED_RED_RCC);

    /* Set LED_PIN to 'output push-pull'. */
    gpio_set_mode(LED_GREEN_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_GREEN_PIN);
    //gpio_set_mode(LED_GREEN_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LED_GREEN_PIN);
    gpio_set_mode(LED_RED_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_RED_PIN);

    // Enable PTU pins clock
    rcc_periph_clock_enable(PTU_MEAS_OUT_RCC);
    rcc_periph_clock_enable(PTU_TEMP_REF1_RCC);
    rcc_periph_clock_enable(PTU_TEMP_REF2_RCC);
    rcc_periph_clock_enable(PTU_TEMP_HUMI_RCC);
    rcc_periph_clock_enable(PTU_TEMP_SENSOR_RCC);
    rcc_periph_clock_enable(PTU_TEMP_ACTIVATION_RCC);

    // Set PTU direction
    gpio_set_mode(PTU_MEAS_OUT_GPIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, PTU_MEAS_OUT_PIN);
    gpio_set_mode(PTU_TEMP_REF1_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_REF1_PIN);
    gpio_set_mode(PTU_TEMP_REF2_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_REF2_PIN);
    gpio_set_mode(PTU_TEMP_HUMI_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_HUMI_PIN);
    gpio_set_mode(PTU_TEMP_SENSOR_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_SENSOR_PIN);
    gpio_set_mode(PTU_TEMP_ACTIVATION_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_ACTIVATION_PIN);

    // Reset pins
    gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
    gpio_clear(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
    gpio_clear(PTU_TEMP_HUMI_GPIO,PTU_TEMP_HUMI_PIN);
    gpio_clear(PTU_TEMP_SENSOR_GPIO,PTU_TEMP_SENSOR_PIN);
    gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
}

void ptu_timer_setup(void) {
    /* Enable TIM2 clock. */
    rcc_periph_clock_enable(PTU_MEAS_OUT_TIMER_RCC);
    rcc_periph_clock_enable(RCC_AFIO);

    /* Reset TIM2 peripheral to defaults. */
    rcc_periph_reset_pulse(PTU_MEAS_OUT_TIMER_RST);

    /* Disable Input Capture. */
    /*timer_ic_disable(PTU_MEAS_OUT_TIMER, TIM_IC1);
    timer_ic_enable(PTU_MEAS_OUT_TIMER, TIM_IC2);
    timer_ic_disable(PTU_MEAS_OUT_TIMER, TIM_IC3);
    timer_ic_disable(PTU_MEAS_OUT_TIMER, TIM_IC4);*/

    /* Disable Output Compare. */
    /*timer_disable_oc_output(PTU_MEAS_OUT_TIMER, TIM_OC1);
    timer_disable_oc_output(PTU_MEAS_OUT_TIMER, TIM_OC2);
    timer_disable_oc_output(PTU_MEAS_OUT_TIMER, TIM_OC3);
    timer_disable_oc_output(PTU_MEAS_OUT_TIMER, TIM_OC4);*/

    /* Timer mode: no divider, edge, count up */

    timer_disable_preload(PTU_MEAS_OUT_TIMER);

    timer_continuous_mode(PTU_MEAS_OUT_TIMER);

    timer_set_mode(PTU_MEAS_OUT_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_period(PTU_MEAS_OUT_TIMER, 0xFFFF);

    //timer_set_prescaler(PTU_MEAS_OUT_TIMER, 1);

    timer_set_clock_division(PTU_MEAS_OUT_TIMER,TIM_CR1_CKD_CK_INT);

    //timer_slave_set_mode(PTU_MEAS_OUT_TIMER,TIM_SMCR_SMS_OFF);

    timer_ic_set_polarity(PTU_MEAS_OUT_TIMER, TIM_IC2, TIM_IC_RISING);

    timer_ic_set_input(PTU_MEAS_OUT_TIMER, TIM_IC2, TIM_IC_IN_TI2);

    timer_ic_set_prescaler(PTU_MEAS_OUT_TIMER, TIM_IC2, TIM_IC_PSC_OFF);

    timer_ic_set_filter(PTU_MEAS_OUT_TIMER, TIM_IC2, TIM_IC_OFF);

    nvic_enable_irq(PTU_MEAS_OUT_TIMER_IRQ);



    timer_ic_enable(PTU_MEAS_OUT_TIMER, TIM_IC2);


    timer_enable_irq(PTU_MEAS_OUT_TIMER, TIM_DIER_CC2IE);


    timer_enable_counter(PTU_MEAS_OUT_TIMER);

    // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    //timer_slave_set_mode(PTU_MEAS_OUT_TIMER, TIM_SMCR_SMS_ECM1);
    // Set the input filter parameters for the external trigger
    //timer_slave_set_filter(PTU_MEAS_OUT_TIMER, TIM_IC_OFF);
    //timer_slave_set_polarity(PTU_MEAS_OUT_TIMER, TIM_ET_RISING);
    // Set the external trigger frequency division ratio.
    //timer_slave_set_prescaler(PTU_MEAS_OUT_TIMER, TIM_IC_PSC_OFF);
    // Set Slave Trigger Source, External Trigger input (ETRF)
    //timer_slave_set_trigger(PTU_MEAS_OUT_TIMER, TIM_SMCR_TS_ETRF);
    //timer_update_on_overflow(PTU_MEAS_OUT_TIMER);

    /* Enable TIM2 interrupt. */
    //nvic_enable_irq(PTU_MEAS_OUT_TIMER_IRQ);

    /* Counter enable. */
    //timer_enable_counter(PTU_MEAS_OUT_TIMER);

    /* Enable Channel 2 compare interrupt to recalculate compare values */
    //timer_enable_irq(PTU_MEAS_OUT_TIMER, TIM_DIER_CC2IE);
}

void tim_setup(void) {
    /* Enable TIM4 clock. */
    rcc_periph_clock_enable(RCC_TIM2);

    /* Reset TIM4 peripheral to defaults. */
    rcc_periph_reset_pulse(RST_TIM2);

    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     * (These are actually default values after reset above, so this call
     * is strictly unnecessary, but demos the api for alternative settings)
     */
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    /*
     * Please take note that the clock source for STM32 timers
     * might not be the raw APB1/APB2 clocks.  In various conditions they
     * are doubled.  See the Reference Manual for full details!
     * Set the prescaler to have the timer run at 100 Hz
     * TIM3 running on 24 kHz now
     */
    timer_set_prescaler(TIM2, 2*rcc_apb1_frequency/24000);

    /* Disable preload. */
    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);

    /* Count to 240, update compare value continuously */
    // count to 240 with 24 kHz clock => 100 Hz overflow
    timer_set_period(TIM2,240);

    /* Set the initual output compare value for OC4. */
    timer_set_oc_value(TIM2,TIM_OC4, 120); // PWD 50% duty cycle
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1); // PWM Output is active (high) when counter is less than output compare value
    timer_enable_oc_output(TIM2, TIM_OC4); // Enable output comparator

    /* Counter enable. */
    timer_enable_counter(TIM2);
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
