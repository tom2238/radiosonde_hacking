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
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/iwdg.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/iwdg.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "si4032.h"
#include "utils.h"

// Marker tones in Hz
#define RS_MARKER_TONE_1_F 1300
#define RS_MARKER_TONE_2_F 1940
#define RS_MARKER_TONE_3_F 2590
// Marker tones length in ms
#define RS_MARKER_TONE_1_T 520
#define RS_MARKER_TONE_2_T 520
#define RS_MARKER_TONE_3_T 1020

#define RS_MARKER_BEEP_SIZE 32

static const uint8_t marker_beep[RS_MARKER_BEEP_SIZE] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // Watchdog if MCU freeze
    watchdog_setup();
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

    int i;

    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);

    console_puts("Start ...\n");

    // TX FIFO Mode with Packet Handler Disabled
    // Infinite packet (over 255 bytes),Tone baud, 5500 Hz deviation, packet size dont care, 80 nibbles dont care
    Si4032_PacketMode(PACKET_TYPE_INFINITE,RS_MARKER_TONE_1_F*2,5500,0xFF,80);
    Si4032_SetFrequency(433.120f);
    // Clear FIFO content on start
    Si4032_ClearFIFO();
    // Add beep
    Si4032_WritePacketData(marker_beep,0,RS_MARKER_BEEP_SIZE);
    // Enable transmission
    Si4032_PacketTx();

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
        Si4032_SetTxPower(SI4032_TX_POWER_11_DBM);

        // Send 169 bytes with speed 1300*2=2600 bds
        for(i=0;i<5;i++) { // 5 * 32 = 160
            // Wait for empty FIFO
            while(!Si4032_IsFIFOEmpty());
            Si4032_SetTxDataRate(RS_MARKER_TONE_1_F*2);
            // Add beep
            Si4032_WritePacketData(marker_beep,0,RS_MARKER_BEEP_SIZE);
        }
        // Add last 9 bytes
        Si4032_WritePacketData(marker_beep,0,9);

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);        
        Si4032_SetTxPower(SI4032_TX_POWER_17_DBM);

        // Send 253 bytes with speed 1940*2=3880 bds
        for(i=0;i<7;i++) { // 7 * 32 = 224
            // Wait for empty FIFO
            while(!Si4032_IsFIFOEmpty());
            Si4032_SetTxDataRate(RS_MARKER_TONE_2_F*2);
            // Add beep
            Si4032_WritePacketData(marker_beep,0,RS_MARKER_BEEP_SIZE);
        }
        // Add last 29 bytes
        while(!Si4032_IsFIFOEmpty());
        Si4032_WritePacketData(marker_beep,0,29);

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
        Si4032_SetTxPower(SI4032_TX_POWER_20_DBM);

        // Send 662 bytes with speed 2590*2=5180 bds
        for(i=0;i<20;i++) { // 20 * 32 = 640
            // Wait for empty FIFO
            while(!Si4032_IsFIFOEmpty());
            Si4032_SetTxDataRate(RS_MARKER_TONE_3_F*2);
            // Add beep
            Si4032_WritePacketData(marker_beep,0,RS_MARKER_BEEP_SIZE);
        }
        // Add last 22 bytes
        while(!Si4032_IsFIFOEmpty());
        Si4032_WritePacketData(marker_beep,0,22);

        // Clear watchdog timer
        iwdg_reset();
	}
	return 0;
}
