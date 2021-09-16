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

/* QtCreator run options for STM32 kit:
 * Executable: st-flash
 * Command line arguments: write main.bin 0x8000000
 * Working directory: %{CurrentProject:BuildPath}
 */

/* Add
 * ../../libopencm3/include
 * ../libraries
 * into <project name>.include file to show libs and function help
 */

/* Vaisala RS41 Upload
 * 1. Press power button to power up MCU
 * 2. Erase with: st-flash erase
 * 3. Upload with: st-flash wrire main.bin 0x8000000
 * 4. st-info --probe output:
 * Found 1 stlink programmers
 * version:    V2J29S7
 * serial:     550035001100005153484c4e
 * hla-serial: "\x55\x00\x35\x00\x11\x00\x00\x51\x53\x48\x4c\x4e"
 * flash:      65536 (pagesize: 1024)
 * sram:       8192
 * chipid:     0x0420
 * descr:      F1xx Value Line
 * In case of problem, try connect under reset
 */

// Global LibOpenCM3 libraries
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "ublox6.h"
#include "utils.h"

void usart3_isr(void);
void usart1_isr(void);

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // LEDs GPIO
    gpio_setup();
    // Systick for delay function
    systick_setup();
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(500);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);
    delay(500);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(500);
    // USART3 for serial print
    usart_setup();
    // USART1 for GPS
    gps_usart_setup(9600);
    delay(10);
    // Intialize ublox
    // TODO: handle ack/nack reply
    Ublox6_Init();
    // Set different leds state
    gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(500);
    console_puts("Init done!\n");

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        //gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
        delay(500);

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        //gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
        delay(500);
	}

	return 0;
}

void usart3_isr(void) {
    /* Check if we were called because of RXNE. */
    if (((USART_CR1(XDATA_USART) & USART_CR1_RXNEIE) != 0) && ((USART_SR(XDATA_USART) & USART_SR_RXNE) != 0)) {
        uint16_t c = usart_recv(XDATA_USART);
        if (c == 'C') {
            gpio_clear(LED_RED_GPIO,LED_RED_PIN);
        } else if (c == 'S'){
            gpio_set(LED_RED_GPIO,LED_RED_PIN);
        }
    }
}

void usart1_isr(void) {
    /* Check if we were called because of RXNE. */
    if (((USART_CR1(GPS_USART) & USART_CR1_RXNEIE) != 0) && ((USART_SR(GPS_USART) & USART_SR_RXNE) != 0)) {
        // ...
        uint16_t c = usart_recv(GPS_USART);
        console_putc(c);
    }
}

