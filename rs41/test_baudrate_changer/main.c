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
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/timer.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/iwdg.h>
#include <libopencm3/stm32/f1/timer.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "si4032.h"
#include "utils.h"

// Marker tones in Hz
#define RS_MARKER_TONE_1_F 1200
#define RS_MARKER_TONE_2_F 2200

#define RS_TONE_SET_SIZE 64
static const uint8_t tone_set[RS_TONE_SET_SIZE] = {
    0,1,1,1,1,1,1,0,
    0,1,0,1,1,1,0,0,
    1,0,0,1,1,0,1,0,
    1,1,0,0,0,1,1,1,
    0,1,1,0,0,0,1,1,
    1,0,0,1,0,1,1,0,
    0,1,0,1,1,0,1,0,
    0,1,1,1,1,1,1,0
};

#define RS_MARKER_BEEP_SIZE 32
static const uint8_t marker_beep[RS_MARKER_BEEP_SIZE] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};

static uint8_t change_en = 1;

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
    // Timer
    tim_setup();
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
    // Infinite packet (over 255 bytes),Tone baud, 4000 Hz deviation, packet size dont care, 80 nibbles dont care
    Si4032_PacketMode(PACKET_TYPE_INFINITE,RS_MARKER_TONE_1_F*2,4000,0xFF,80);
    Si4032_SetFrequency(433.120f);
    Si4032_SetTxPower(SI4032_TX_POWER_1_DBM);
    // Clear FIFO content on start
    Si4032_ClearFIFO();
    // Add beep
    Si4032_WritePacketData(marker_beep,0,RS_MARKER_BEEP_SIZE);
    Si4032_WritePacketData(marker_beep,0,20);
    // Enable transmission
    Si4032_PacketTx();

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);

        if(change_en == 1) {
            // Wait for empty FIFO
            while(!Si4032_IsFIFOEmpty());
            // Add beep
            Si4032_WritePacketData(marker_beep,0,RS_MARKER_BEEP_SIZE);
            Si4032_WritePacketData(marker_beep,0,20);
        }
        delay(100);


        // Clear watchdog timer
        iwdg_reset();
	}
	return 0;
}

void tim2_isr(void) {
    static unsigned int tone_sel = 0;
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        if(change_en == 1) {
            change_en = 0;
            if(tone_set[tone_sel] == 0) {
                Si4032_SetTxDataRate(RS_MARKER_TONE_1_F*2);

            } else {
                Si4032_SetTxDataRate(RS_MARKER_TONE_2_F*2);
            }
            tone_sel++;
            if(tone_sel >= RS_TONE_SET_SIZE) {
                tone_sel = 0;
            }
            //if(Si4032_IsFIFOEmpty()) {
            //    Si4032_WritePacketData(marker_beep,0,1);
            //}
            gpio_toggle(LED_RED_GPIO,LED_RED_PIN);
            change_en = 1;
        }
    }
}
