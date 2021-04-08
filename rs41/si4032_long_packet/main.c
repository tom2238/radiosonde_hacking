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
#include "frame.h"

// FSK frame
static FrameData dataframe;

// MCU supply voltage
static uint16_t adc_supply = 0;

// Main.c static functions
static void FrameCalculate(FrameData *frame, unsigned int count, uint16_t adc_vref, uint16_t adc_ch0, uint16_t adc_temp);
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
    uint16_t adc_val;
    uint16_t adc_vref;
    int adc_temperature;

    // Frame infos
    unsigned int framecount = 0;
    int i,j;

    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);

    console_puts("Start ...\n");

    // 1200 baud, 4800 Hz deviation, 128 bytes packet size
    Si4032_PacketMode(4800,2400,FRAME_USER_LEN+CRC_SIZE);

    // Packet bytes to send
    uint8_t packet_data[FRAME_USER_LEN+CRC_SIZE] = {0};

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        delay(2500);

        // Print frame counter
        console_putc('[');
        console_print_int(framecount);
        console_putc(']');

        // Read ADC supply voltage
        adc_supply = read_adc_supply()/4;
        adc_supply += read_adc_supply()/4;
        adc_supply += read_adc_supply()/4;
        adc_supply += read_adc_supply()/4;
        console_puts(", ADC supply: ");
        console_print_float(((float)(adc_supply))/1000);
        console_puts(" V, ");

        // Reading adc voltage reference
        adc_vref = read_adc_voltage(ADC_CHANNEL_VREF)/4;
        adc_vref += read_adc_voltage(ADC_CHANNEL_VREF)/4;
        adc_vref += read_adc_voltage(ADC_CHANNEL_VREF)/4;
        adc_vref += read_adc_voltage(ADC_CHANNEL_VREF)/4;
        console_puts("ADC reference: ");
        console_print_float(((float)(adc_vref))/1000);
        console_puts(" V, ");

        // Reading from PA5 pin, battery voltage
        // Divide 4 to get average from 4 samples
        // Multiply 2 to get correct value from voltage divider
        adc_val = 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        adc_val += 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        adc_val += 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        adc_val += 2*read_adc_voltage(VBAT_MON_ADC_CHANNEL)/4;
        console_puts("Battery voltage: ");
        console_print_float(((float)(adc_val))/1000);
        console_puts(" V, ");

        // Reading MCU temperature
        adc_temperature = read_adc_temperature();
        console_puts("MCU temperature: ");
        console_print_float(((float)(adc_temperature))/100);
        console_puts(" °C\n");

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        delay(100);

        // New packet
        dataframe = NewFrameData(FRAME_USER_LEN + HEAD_SIZE + ECC_SIZE + CRC_SIZE, FRAME_MOD_NRZ);
        // Calculate new frame data
        FrameCalculate(&dataframe,framecount,adc_vref,adc_val,adc_temperature);     
        j=0;
        // Copy frame data + crc into packet, without header
        for(i=HEAD_SIZE;i<FRAME_USER_LEN+HEAD_SIZE+CRC_SIZE;i++) {
            packet_data[j] = dataframe.value[i];
            j++;
        }
        // Preambule and header is added in Si4032
        //Si4032_WritePacket(packet_data,FRAME_USER_LEN+CRC_SIZE);
        //while(!Si4032_IsPacketSent());

        // Clear FIFO content on start
        Si4032_ClearFIFO();
        // Write 64 bytes into FIFO
        Si4032_WritePacketData(packet_data,0,64);
        // Read and clear interrupt flags
        Si4032_ClearInterruptStatus();
        // Enable FIFO empty interrupt
        Si4032_EnableFIFOEmptyInterrupt();
        // Enable transmission
        Si4032_PacketTx();
        // Wait for empty FIFO
        while(!Si4032_IsFIFOEmpty());
        // Write 32 bytes into FIFO
        Si4032_WritePacketData(packet_data,64,32);
        // Wait for empty FIFO
        while(!Si4032_IsFIFOEmpty());
        // Write 32 bytes into FIFO
        Si4032_WritePacketData(packet_data,96,32);
        // Enable packet send interrupt
        Si4032_EnablePacketSentInterrupt();
        // Wait for send
        while(!Si4032_IsPacketSent());
        Si4032_DisableTx();
        // Increment frame counter
        framecount++;
        // Clear watchdog timer
        iwdg_reset();
	}

	return 0;
}

static void FrameCalculate(FrameData *frame, unsigned int count, uint16_t adc_vref, uint16_t adc_ch0, uint16_t adc_temp){
    // Current frame number
    frame->value[8] = (count >> 24) & 0xFF;
    frame->value[9] = (count >> 16) & 0xFF;
    frame->value[10] = (count >> 8) & 0xFF;
    frame->value[11] = (count >> 0) & 0xFF;
    // ADC Vref voltage in milivolts
    frame->value[12] = (adc_vref >> 8) & 0xFF;
    frame->value[13] = (adc_vref >> 0) & 0xFF;
    // ADC voltage on channel 0, pin PA0 in milivolts
    frame->value[14] = (adc_ch0 >> 8) & 0xFF;
    frame->value[15] = (adc_ch0 >> 0) & 0xFF;
    // ADC MCU temperature in centi celsius degree
    frame->value[16] = (adc_temp >> 8) & 0xFF;
    frame->value[17] = (adc_temp >> 0) & 0xFF;
    // ADC MCU supply voltage in milivolts
    frame->value[18] = (adc_supply >> 8) & 0xFF;
    frame->value[19] = (adc_supply >> 0) & 0xFF;
    // Calculate CRC16
    CalculateCRC16(frame);
    FrameXOR(frame,0); // XORing NRZ frame
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

