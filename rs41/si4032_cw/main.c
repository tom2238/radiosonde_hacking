/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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

/* Add ../../libopencm3/include into <project name>.include file to show libs and function help */

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
#include "uart.h"
#include "itoa.h"

int main(void) {
    // Setup registers
    clock_setup();
    gpio_setup();
    usart_setup();
    systick_setup();
    spi_setup();
    Si4032_Init();
    //spi_setup();
    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);

    //Si4032_Init2();
    Si4032_EnableTx();
    console_puts("R1250002 start ...\nVersion Si4032: ");

    char buf[64];
    uint8_t version = Si4032_GetVersion();
    itoa(version,buf,10);
    console_puts(buf);
    console_puts("\n");

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
        Si4032_EnableTx();

        delay(400);

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
        Si4032_DisableTx();

        delay(600);
        console_puts("Vaisala RS41: ");
        uint16_t siadc = Si4032_GetBatteryVoltage();
        itoa(siadc,buf,10);
        console_puts(buf);
        console_puts(" mV\n");
	}

	return 0;
}
