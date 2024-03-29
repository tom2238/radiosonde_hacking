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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/timer.h>
#include <libopencm3/stm32/f1/spi.h>
#include <libopencm3/stm32/f1/usart.h>
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
    gpio_set_mode(LED_GREEN_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_GREEN_PIN);
    gpio_set_mode(LED_RED_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_RED_PIN);
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
