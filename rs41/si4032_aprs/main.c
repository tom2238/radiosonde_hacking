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
#include "ax25.h"

#define APRS_CALLSIGN "N0CALL"
#define APRS_SSID 'B'
#define TXT_BUFFER "!4904.91N/01649.04E_000/000g000t049b10120h85; 09,7C;HW:PIC3COM,QTH:Otnice"

// MCU supply voltage
static uint16_t adc_supply = 0;

// Main.c static functions
static uint16_t read_adc(uint8_t channel);
static int read_adc_temperature(void);
static uint16_t read_adc_supply(void);
static uint16_t read_adc_voltage(uint8_t channel);

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // Watchdog if MCU freeze
    watchdog_setup();
    // LEDs GPIO
    gpio_setup();
    // Setup ADC1 for temperaure, vref, and PA5 pin
    adc_setup();
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

    // Read ADC supply voltage
    adc_supply = read_adc_supply();
    // ADC variables
    uint16_t adc_val = 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL);
    uint16_t adc_vref = read_adc_voltage(ADC_CHANNEL_VREF);
    int adc_temperature = read_adc_temperature();

    // Frame infos
    unsigned int framecount = 0;
    int i,j;

    // Millis timer delay
    uint64_t millis_last = millis();

    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);

    console_puts("Start ...\n");

    // APRS, 13200 sample rate, common for 1200 Hz and 2200 Hz frequencies
    // TX FIFO Mode with Packet Handler Disabled
    // Infinite packet (over 255 bytes), 13200 baud, 2750 Hz deviation, packet size dont care, 80 nibbles dont care
    Si4032_PacketMode(PACKET_TYPE_INFINITE,AX25_SAMPLE_RATE,2750,0xFF,80);
    Ax25_Init((char*)APRS_CALLSIGN,(const uint8_t)APRS_SSID,(char*)"APZQAP",'0',(char*)"WIDE1-1,WIDE2-1");

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        // Wait for 1000 ms, including reading+packet transmitting
        while (millis() < millis_last);

        // Save millis
        millis_last = millis() + 4000;

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        Ax25_SendUIFrameBlocking(TXT_BUFFER);

        // Calculate new frame values

        // Read ADC supply voltage
        adc_supply = read_adc_supply()/4;
        adc_supply += read_adc_supply()/4;
        adc_supply += read_adc_supply()/4;
        adc_supply += read_adc_supply()/4;

        // Reading adc voltage reference
        adc_vref = read_adc_voltage(ADC_CHANNEL_VREF)/4;
        adc_vref += read_adc_voltage(ADC_CHANNEL_VREF)/4;
        adc_vref += read_adc_voltage(ADC_CHANNEL_VREF)/4;
        adc_vref += read_adc_voltage(ADC_CHANNEL_VREF)/4;

        // Reading from PA5 pin, battery voltage
        // Divide 4 to get average from 4 samples
        // Multiply 2 to get correct value from voltage divider
        adc_val = 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        adc_val += 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        adc_val += 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        adc_val += 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;

        // Reading MCU temperature
        adc_temperature = read_adc_temperature();
        // Increment frame counter
        framecount++; 
        // Clear watchdog timer
        iwdg_reset();
	}
	return 0;
}

// ADC reading function
static uint16_t read_adc(uint8_t channel) {
    adc_set_sample_time(ADC1, channel, ADC_SMPR_SMP_239DOT5CYC);
    adc_set_regular_sequence(ADC1,1,&channel);
    adc_start_conversion_direct(ADC1);
    while(!adc_eoc(ADC1));
    return adc_read_regular(ADC1);
}

// ADC reading temperature
static int read_adc_temperature(void) {
    static const int v25 = 141; // 1.41 volt at 25 degC
    int vtemp;
    vtemp = (int)read_adc(ADC_CHANNEL_TEMP) * adc_supply/ 4095;
    return (v25 - vtemp) / 45 + 2500;
}

// Read MCU supply voltage
static uint16_t read_adc_supply(void) {
    // STM32F100 ADC Vref is 1200 mV
    return 1200 * 4095/read_adc(ADC_CHANNEL_VREF); // in milivolts
}

// Return voltage from ADC channel
static uint16_t read_adc_voltage(uint8_t channel) {
    return read_adc(channel) * adc_supply/4095;
}

