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
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "si4032.h"
#include "utils.h"

// Z31915-TX
// Pulse with, all in us
#define WEATHER_SENSOR_PULSE_HIGH 750-45
#define WEATHER_SENSOR_DELAY_ZERO 1750-45
#define WEATHER_SENSOR_DELAY_ONE 4000-45
#define WEATHER_SENSOR_DELAY_SYNC 7750-45
#define WEATHER_SENSOR_CRC_INIT 0
#define WEATHER_SENSOR_CRC_POLY 0x13
#define WEATHER_SENSOR_BYTES_SIZE 5

// Sensor message
static uint8_t sensor_msg[WEATHER_SENSOR_BYTES_SIZE];
static uint8_t sensor_crc[WEATHER_SENSOR_BYTES_SIZE];

static void SendOne(void);
static void SendZero(void);
static void SendSync(void);
static void SendSBit(void);
static uint8_t crc4(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init);
static void SendWeatherSensor(void);

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
    // Delay timers setup
    delay_setup();

    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);

    uint8_t delay_counter = 0;
    // Original received data
    // i -> ID - 8 bit
    // f -> FCS - CRC4
    // rr -> Trend, up, down, stable
    // t -> Temperature in (X°F + 90°F)*10
    // h -> humidity in BCD format
    // c -> channel
    // iiiiiiii ffff ?? rr tttttttttttt hhhhhhhh ?? cc
    // 10100000 1011 00 10 010110111100 00110001 00 11

    Si4032_SetFrequency(433.9059f);
    Si4032_SetModulatioType(SI4032_TX_MODULATION_TYPE_UNMOD);
    Si4032_SetTxPower(SI4032_TX_POWER_5_DBM);
    Si4032_DisableTx();

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
            SendWeatherSensor();
            SendWeatherSensor();
            SendWeatherSensor();
            SendWeatherSensor();
            delay_counter = 0;
            Si4032_DisableTx();
        }
	}
	return 0;
}

static void SendOne(void) {
    Si4032_EnableTx();
    delay_us(WEATHER_SENSOR_PULSE_HIGH);
    Si4032_InhibitTx();
    delay_us(WEATHER_SENSOR_DELAY_ONE);
}

static void SendZero(void) {
    Si4032_EnableTx();
    delay_us(WEATHER_SENSOR_PULSE_HIGH);
    Si4032_InhibitTx();
    delay_us(WEATHER_SENSOR_DELAY_ZERO);
}

static void SendSync(void) {
    Si4032_EnableTx();
    delay_us(1200);
    Si4032_InhibitTx();
    delay_us(800);
}

static void SendSBit(void) {
    Si4032_EnableTx();
    delay_us(WEATHER_SENSOR_PULSE_HIGH);
    Si4032_InhibitTx();
    delay_us(WEATHER_SENSOR_DELAY_SYNC);
}

static uint8_t crc4(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init) {
    unsigned remainder = init << 4; // LSBs are unused
    unsigned poly = polynomial << 4;
    unsigned bit;

    while (nBytes--) {
        remainder ^= *message++;
        for (bit = 0; bit < 8; bit++) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ poly;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder >> 4 & 0x0f; // discard the LSBs
}

/**
 * @brief SendWeatherSensor ID 155, Temperature 32°F, Humidity 32%
 */
static void SendWeatherSensor(void) {
    int i,j;
    uint8_t sensor_msg_crc;
    sensor_msg[0] = 155; // ID
    //                ffff00rr
    sensor_msg[1] = 0b00000000;
    // Temperature, X °F(meas) + 90 °F(offset), 12 bit number, 122+90
    //                tttttttt
    sensor_msg[2] = 0b01001100; // 8 MSB bits
    //                tttthhhh
    sensor_msg[3] = 0b01000011; // Humidity in BCD, 3x %
    //                hhhh00cc
    sensor_msg[4] = 0b00000010; // Humidity in BCD, x0 % , channel 2

    // Calc crc
    for(i=0;i<WEATHER_SENSOR_BYTES_SIZE;i++) {
        sensor_crc[i] = sensor_msg[i];
    }
    // for CRC computation, channel bits are at the CRC position(!)
    sensor_crc[1] = (sensor_crc[1] & 0x0F) | (sensor_crc[4] & 0x0F) << 4;
    // crc4() only works with full bytes
    sensor_msg_crc = crc4(sensor_crc, WEATHER_SENSOR_BYTES_SIZE-1, WEATHER_SENSOR_CRC_POLY, WEATHER_SENSOR_CRC_INIT); // Koopmann 0x9, CCITT-4; FP-4; ITU-T G.704
    sensor_msg_crc ^= sensor_crc[4] >> 4; // last nibble is only XORed
    // Save CRC
    sensor_msg[1] |= (sensor_msg_crc << 4) & 0xF0;
    /*
    // Overwrite test
    // 10100000 1011 00 10 010110111100 00110001 00 11
    sensor_msg[0] = 0b10100000;
    sensor_msg[1] = 0b10110010;
    sensor_msg[2] = 0b01011011;
    sensor_msg[3] = 0b11000011;
    sensor_msg[4] = 0b00010011;
    */
    // S delay
    Si4032_InhibitTx();
    delay_us(WEATHER_SENSOR_DELAY_SYNC);
    // Send sync
    SendSync();
    SendSync();
    SendSync();
    SendSync();
    SendSBit();
    // Send data
    for(i=0;i<WEATHER_SENSOR_BYTES_SIZE;i++) {
        for(j=7;j>=0;j--) {
            if((sensor_msg[i] >> j) & 0x01) {
                SendOne();
            } else {
                SendZero();
            }
        }
    }
    // End
    SendSBit();
}
