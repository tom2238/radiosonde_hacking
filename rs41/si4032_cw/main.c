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
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "si4032.h"
#include "utils.h"
#include "morse.h"

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // LEDs GPIO
    gpio_setup();
    // Systick for delay function
    systick_setup();
    // USART for serial print
    usart_setup();
    // SPI config
    spi_setup();
    // Wait at least 15ms before any initialization SPI commands are sent to the radio
    delay(20);
    // Inicialize Si4032
    Si4032_Init();
    // For some reason we have to do this again
    delay(20);
    Si4032_Init2();

    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);

    // Message buffer
    char buf[64];
    uint8_t delay_counter = 0;
    unsigned int packet_counter = 0;

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        delay(100);

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        delay(900);

        delay_counter++;
        if(delay_counter > 15) {// cca after 15 seconds
            itoa(packet_counter,buf,10);
            Morse_SendMessage("EES,RS41,PACKET,",MORSE_DEFAULT_WPM);
            Morse_SendMessage(buf,MORSE_DEFAULT_WPM);
            Morse_SendMessage(",SEE",MORSE_DEFAULT_WPM);
            delay_counter = 0;
            packet_counter++;
        }
	}

	return 0;
}
