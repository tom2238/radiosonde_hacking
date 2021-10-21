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
#include "si4032.h"
#include "ptu_measure.h"

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // Systick for delay function
    systick_setup();
    // LEDs GPIO
    gpio_setup();
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
    // Inicialize PTU
    PTU_Init();

    Si4032_DisableTx();
    PTU_DisableReferenceHeating();

    // Set leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);

    console_puts("Start ...\n");

    PTURAWData raw_ptu;

	while (1) {
        /* Blink the LED on the board. */   
        gpio_clear(LED_RED_GPIO,LED_RED_PIN);

        PTU_MeasureTemperature(&raw_ptu);

        /* Blink the LED on the board. */
        gpio_set(LED_RED_GPIO,LED_RED_PIN);

        console_puts("T_REF1: Freq: ");
        console_print_int(raw_ptu.temperature_ref1);
        console_puts(" Hz\n");

        console_puts("T_Humidity: Freq: ");
        console_print_int(raw_ptu.temperature_humi);
        console_puts(" Hz\n");

        console_puts("T_Sensor: Freq: ");
        console_print_int(raw_ptu.temperature_sensor);
        console_puts(" Hz\n");

        console_puts("T_REF2: Freq: ");
        console_print_int(raw_ptu.temperature_ref2);
        console_puts(" Hz\n");

        /* Blink the LED on the board. */
        gpio_clear(LED_RED_GPIO,LED_RED_PIN);

        PTU_MeasureHumidity(&raw_ptu);

        /* Blink the LED on the board. */
        gpio_set(LED_RED_GPIO,LED_RED_PIN);

        console_puts("H_REF1: Freq: ");
        console_print_int(raw_ptu.humidity_ref1);
        console_puts(" Hz\n");

        console_puts("H_Sensor: Freq: ");
        console_print_int(raw_ptu.humidity_sensor);
        console_puts(" Hz\n");

        console_puts("H_REF2: Freq: ");
        console_print_int(raw_ptu.humidity_ref2);
        console_puts(" Hz\n");

        /* Blink the LED on the board. */
        gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
        delay(800);
        gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);
	}
	return 0;
}

