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
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/spi.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/timer.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "si4032.h"

void gpio_setup(void) {
    /* Enable the correct clock. */
    /* Enable GPIO RCC clock. */
    rcc_periph_clock_enable(LED_GREEN_RCC);
    rcc_periph_clock_enable(LED_RED_RCC);

    /* Set LED_PIN to 'output push-pull'. */
    gpio_set_mode(LED_GREEN_GPIO, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_GREEN_PIN);
    gpio_set_mode(LED_RED_GPIO, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_RED_PIN);

    // Enable PTU pins clock
    // Temperature
    rcc_periph_clock_enable(PTU_TEMP_REF1_RCC);
    rcc_periph_clock_enable(PTU_TEMP_REF2_RCC);
    rcc_periph_clock_enable(PTU_TEMP_HUMI_RCC);
    rcc_periph_clock_enable(PTU_TEMP_SENSOR_RCC);
    rcc_periph_clock_enable(PTU_TEMP_ACTIVATION_RCC);
    // Humidity
    rcc_periph_clock_enable(PTU_HUMI_REF1_RCC);
    rcc_periph_clock_enable(PTU_HUMI_REF2_RCC);
    rcc_periph_clock_enable(PTU_HUMI_SENSOR_RCC);
    rcc_periph_clock_enable(PTU_HUMI_ACTIVATION_RCC);

    // Set PTU direction
    // Temperature
    gpio_set_mode(PTU_TEMP_REF1_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_REF1_PIN);
    gpio_set_mode(PTU_TEMP_REF2_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_REF2_PIN);
    gpio_set_mode(PTU_TEMP_HUMI_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_HUMI_PIN);
    gpio_set_mode(PTU_TEMP_SENSOR_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_SENSOR_PIN);
    gpio_set_mode(PTU_TEMP_ACTIVATION_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_ACTIVATION_PIN);
    // Humidity
    gpio_set_mode(PTU_HUMI_REF1_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_REF1_PIN);
    gpio_set_mode(PTU_HUMI_REF2_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_REF2_PIN);
    gpio_set_mode(PTU_HUMI_SENSOR_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_SENSOR_PIN);
    gpio_set_mode(PTU_HUMI_ACTIVATION_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_ACTIVATION_PIN);

    // Reset PTU pins
    // Temperature
    gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
    gpio_clear(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
    gpio_clear(PTU_TEMP_HUMI_GPIO,PTU_TEMP_HUMI_PIN);
    gpio_clear(PTU_TEMP_SENSOR_GPIO,PTU_TEMP_SENSOR_PIN);
    gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
    // Humidity
    gpio_clear(PTU_HUMI_REF1_GPIO,PTU_HUMI_REF1_PIN);
    gpio_clear(PTU_HUMI_REF2_GPIO,PTU_HUMI_REF2_PIN);
    gpio_clear(PTU_HUMI_SENSOR_GPIO,PTU_HUMI_SENSOR_PIN);
    gpio_clear(PTU_HUMI_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
}

void ptu_timer_setup(void) {
    // Input capture timer setup
    // Enable clock
    rcc_periph_clock_enable(PTU_MEAS_OUT_TIMER_RCC);
    // Reset peripheral
    rcc_periph_reset_pulse(PTU_MEAS_OUT_TIMER_RST);
    // Clock prescaler is 0, maximum accuracy
    timer_set_prescaler(PTU_MEAS_OUT_TIMER,0);
    // Counting up
    timer_direction_up(PTU_MEAS_OUT_TIMER);
    // Typically, set maximum period
    timer_set_period(PTU_MEAS_OUT_TIMER,PTU_MEAS_OUT_TIMER_PERIOD);
    // No clock divider
    timer_set_clock_division(PTU_MEAS_OUT_TIMER,TIM_CR1_CKD_CK_INT);
    // Enable Auto-Reload Buffering.
    timer_enable_preload(PTU_MEAS_OUT_TIMER);
    // Reset master
    timer_set_master_mode(PTU_MEAS_OUT_TIMER,TIM_CR2_MMS_RESET);
    // Disable slave mode
    timer_slave_set_mode(PTU_MEAS_OUT_TIMER,TIM_SMCR_SMS_OFF);
    // Rising edge
    timer_ic_set_polarity(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_RISING);
    // Input on PA1 pin, channel 2
    timer_ic_set_input(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_IN_TI2);
    // IC prescaler is off
    timer_ic_set_prescaler(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_PSC_OFF);
    // IC lowpass filter is TimerClock/(8*8)
    timer_ic_set_filter(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_DTF_DIV_8_N_8);
    // Enable input capture
    timer_ic_enable(PTU_MEAS_OUT_TIMER,TIM_IC2);
    // Enable IRQ from timer
    nvic_enable_irq(PTU_MEAS_OUT_TIMER_IRQ);
    // Enable timer counter
    timer_enable_counter(PTU_MEAS_OUT_TIMER);
    // Enable channel 2 compare interrupt
    timer_enable_irq(PTU_MEAS_OUT_TIMER, TIM_DIER_CC2IE);

    // Frequency timer setup
    // Enable clock
    rcc_periph_clock_enable(FREQUENCY_TIMER_RCC);
    // Reset peripheral
    rcc_periph_reset_pulse(FREQUENCY_TIMER_RST);
    // No clock divider, edge alignment, counting up
    timer_set_mode(FREQUENCY_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // Clock prescaler is 0, maximum accuracy
    timer_set_prescaler(FREQUENCY_TIMER,0);
    // Disable Auto-Reload Buffering.
    timer_disable_preload(FREQUENCY_TIMER);
    // Enable the Timer to Run Continuously.
    timer_continuous_mode(FREQUENCY_TIMER);
    // Typically, set maximum period
    timer_set_period(FREQUENCY_TIMER, FREQUENCY_TIMER_PERIOD);
    // Enable IRQ from timer
    nvic_enable_irq(FREQUENCY_TIMER_IRQ);
    // Enable timer overflow interrupt
    timer_enable_irq(FREQUENCY_TIMER, TIM_DIER_UIE);
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

void spi_setup(void) {
  /* Enable clocks */
  rcc_periph_clock_enable(SI4032_NSEL_RCC);
  rcc_periph_clock_enable(SI4032_SPI_PORT_SCK_RCC);
  rcc_periph_clock_enable(SI4032_SPI_PORT_MOSI_RCC);
  rcc_periph_clock_enable(SI4032_SPI_PORT_MISO_RCC);
  /* Enable SPI2 Periph and gpio clocks */
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(SI4032_SPI_PORT_RCC);

  /* GPIO NSEL pin */
  gpio_set_mode(SI4032_NSEL_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SI4032_NSEL_PIN);
  /* SPI SCL and MOSI pin */
  gpio_set_mode(SI4032_SPI_PORT_SCK_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, SI4032_SPI_PORT_SCK_PIN);
  gpio_set_mode(SI4032_SPI_PORT_MOSI_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, SI4032_SPI_PORT_MOSI_PIN);
  /* SPI MISO pin */
  gpio_set_mode(SI4032_SPI_PORT_MISO_GPIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SI4032_SPI_PORT_MISO_PIN);

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SI4032_SPI_PORT);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/16 of peripheral clock frequency
   * Clock polarity: Idle Low
   * Clock phase: Data valid on 1st clock pulse
   * Data frame format: 16-bit
   * Frame format: MSB First
   */

  spi_init_master(SI4032_SPI_PORT, SI4032_SPI_CNF_RATE, SI4032_SPI_CNF_CLOCK_POL,
                  SI4032_SPI_CNF_CLOCK_PHASE, SI4032_SPI_CNF_DATASIZE, SI4032_SPI_CNF_ENDIAN);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SI4032_SPI_PORT);
  spi_set_nss_high(SI4032_SPI_PORT);

  /* Set full duplex, two wires */
  spi_set_unidirectional_mode(SI4032_SPI_PORT);

  /* Disable CRC */
  spi_disable_crc(SI4032_SPI_PORT);

  /* Enable SPI1 periph. */
  spi_enable(SI4032_SPI_PORT);
}
