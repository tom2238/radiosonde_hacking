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

#ifndef STM32_INIT_C
#define STM32_INIT_C

// Vaisala RS41 leds
#define LED_GREEN_GPIO GPIOB
#define LED_GREEN_PIN GPIO7
#define LED_GREEN_RCC RCC_GPIOB
#define LED_RED_GPIO GPIOB
#define LED_RED_PIN GPIO8
#define LED_RED_RCC RCC_GPIOB

// Vaisala RS41 Si4032 connection
// Chip select
#define SI4032_NSEL_GPIO GPIOC
#define SI4032_NSEL_PIN GPIO13
#define SI4032_NSEL_RCC RCC_GPIOC
// SPI interface
#define SI4032_SPI_PORT SPI2
#define SI4032_SPI_PORT_RCC RCC_SPI2
// Also known as SDI
#define SI4032_SPI_PORT_MOSI_GPIO GPIOB
#define SI4032_SPI_PORT_MOSI_PIN GPIO15
#define SI4032_SPI_PORT_MOSI_RCC RCC_GPIOB
// Also known as SDO
#define SI4032_SPI_PORT_MISO_GPIO GPIOB
#define SI4032_SPI_PORT_MISO_PIN GPIO14
#define SI4032_SPI_PORT_MISO_RCC RCC_GPIOB
// SPI Clock
#define SI4032_SPI_PORT_SCK_GPIO GPIOB
#define SI4032_SPI_PORT_SCK_PIN GPIO13
#define SI4032_SPI_PORT_SCK_RCC RCC_GPIOB
// SPI config
#define SI4032_SPI_CNF_RATE SPI_CR1_BAUDRATE_FPCLK_DIV_16
#define SI4032_SPI_CNF_CLOCK_POL SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE
#define SI4032_SPI_CNF_CLOCK_PHASE SPI_CR1_CPHA_CLK_TRANSITION_1
#define SI4032_SPI_CNF_DATASIZE SPI_CR1_DFF_16BIT
#define SI4032_SPI_CNF_ENDIAN SPI_CR1_MSBFIRST

// Vaisala RS41 XDATA UART
#define XDATA_USART USART3
#define XDATA_USART_GPIO GPIOB
#define XDATA_USART_RCC_GPIO RCC_GPIOB
#define XDATA_USART_RCC RCC_USART3

void gpio_setup(void);
void usart_setup(void) ;
void clock_setup(void);
void systick_setup(void);
uint64_t millis(void);
void delay(uint64_t duration);
void spi_setup(void);

#endif // STM32_INIT_C
