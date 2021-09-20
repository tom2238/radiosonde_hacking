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
#include "si4032.h"
#include "frame.h"

void usart3_isr(void);
void usart1_isr(void);
static void FrameCalculate(FrameData *frame, uBlox6_GPSData *gps);

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // LEDs GPIO
    gpio_setup();
    // Systick for delay function
    systick_setup();
    // SPI config
    spi_setup();
    // Wait at least 15ms before any initialization SPI commands are sent to the radio
    delay(20);
    // Inicialize Si4032
    Si4032_Init();
    // For some reason we have to do this again
    delay(20);
    Si4032_Init2();

    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(500);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);
    delay(500);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(500);

    // USART3 for serial print
    usart_setup();
    // Intialize ublox
    Ublox6_Init();
    // Set different leds state
    gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(500);

    // Frame init
    Frame_Init(64,35,FRAME_MOD_NRZ);

    // Short packet (<= 64), 4800 baud, 3600 Hz deviation, 22 bytes packet size, 80 nibbles
    Si4032_PacketMode(PACKET_TYPE_SHORT,4800,2400,Frame_GetUserLength()+Frame_GetCRCSize(),80);

    uBlox6_GPSData gpsData;
    // Millis timer delay
    uint64_t millis_last = millis();
    // FSK frame
    static FrameData dataframe;

    console_puts("Init done!\n");

	while (1) {
        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);

        // Wait for 1000 ms
        while (millis() < millis_last);
        // Save millis
        millis_last = millis() + 1000;

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);        
        delay(200);

        Ublox6_GetLastData(&gpsData);

        console_print_int(gpsData.year);
        console_puts(".");
        console_print_int(gpsData.month);
        console_puts(".");
        console_print_int(gpsData.day);
        console_puts(". ");
        console_print_int(gpsData.hour);
        console_puts(":");
        console_print_int(gpsData.min);
        console_puts(":");
        console_print_int(gpsData.sec);
        //console_puts(" CK_R");
        //console_print_int(gpsData.ck_rec_a);
        //console_puts(",");
        //console_print_int(gpsData.ck_rec_b);
        //console_puts(", CK_C");
        //console_print_int(gpsData.ck_cal_a);
        //console_puts(",");
        //console_print_int(gpsData.ck_cal_b);
        //console_puts(", PS");
        //console_print_int(gpsData.pay_size);
        console_puts("\n");
        /*console_puts(", Fix:");
        console_print_int(gpsData.gpsFix);
        console_puts(", SVs:");
        console_print_int(gpsData.numSV);
        console_puts(", Lat:");
        console_print_int(gpsData.lat);
        console_puts(", Lon:");
        console_print_int(gpsData.lon);
        console_puts(", H:");
        console_print_int(gpsData.heading);
        console_puts(", gS:");
        console_print_int(gpsData.gSpeed);
        console_puts(", S:");
        console_print_int(gpsData.speed);
        console_puts("\n");*/

        // New packet
        dataframe = Frame_NewData(Frame_GetUserLength() + Frame_GetHeadSize() + Frame_GetECCSize() + Frame_GetCRCSize(), Frame_GetCoding());
        // Calculate new frame data
        FrameCalculate(&dataframe,&gpsData);
        // Calculate CRC16
        Frame_CalculateCRC16(&dataframe);
        Frame_XOR(&dataframe,0); // XORing NRZ frame
        // Preamble and header is added in Si4032
        Si4032_WriteShortPacket((((uint8_t*)&dataframe.value) + Frame_GetHeadSize()), Frame_GetUserLength()+Frame_GetCRCSize());
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
        uint16_t c = usart_recv(GPS_USART);
        //console_putc(c);
        //Ublox6_HandleByte((uint8_t)c);
        //Ublox6_HandleByte2((uint8_t)c);
        Ublox6_HandleByte3((uint8_t)c);
    }
}

static void FrameCalculate(FrameData *frame, uBlox6_GPSData *gps) {
    // GPS TIME UTC
    frame->value[8] = (gps->year >> 24) & 0xFF;
    frame->value[9] = (gps->year >> 16) & 0xFF;
    frame->value[10] = (gps->year >> 8) & 0xFF;
    frame->value[11] = (gps->year >> 0) & 0xFF;
    frame->value[12] = gps->month;
    frame->value[13] = gps->day;
    frame->value[14] = gps->hour;
    frame->value[15] = gps->min;
    frame->value[16] = gps->sec;
    // GPS NAV SOL
    frame->value[17] = gps->gpsFix;
    frame->value[18] = gps->numSV;
    // GPS NAV POSLLH
    frame->value[19] = (gps->lon >> 24) & 0xFF;
    frame->value[20] = (gps->lon >> 16) & 0xFF;
    frame->value[21] = (gps->lon >> 8) & 0xFF;
    frame->value[22] = (gps->lon >> 0) & 0xFF;
    frame->value[23] = (gps->lat >> 24) & 0xFF;
    frame->value[24] = (gps->lat >> 16) & 0xFF;
    frame->value[25] = (gps->lat >> 8) & 0xFF;
    frame->value[26] = (gps->lat >> 0) & 0xFF;
    frame->value[27] = (gps->hMSL >> 24) & 0xFF;
    frame->value[28] = (gps->hMSL >> 16) & 0xFF;
    frame->value[29] = (gps->hMSL >> 8) & 0xFF;
    frame->value[30] = (gps->hMSL >> 0) & 0xFF;
    // GPS NAV VELNED
    frame->value[31] = (gps->speed >> 24) & 0xFF;
    frame->value[32] = (gps->speed >> 16) & 0xFF;
    frame->value[33] = (gps->speed >> 8) & 0xFF;
    frame->value[34] = (gps->speed >> 0) & 0xFF;
    frame->value[35] = (gps->gSpeed >> 24) & 0xFF;
    frame->value[36] = (gps->gSpeed >> 16) & 0xFF;
    frame->value[37] = (gps->gSpeed >> 8) & 0xFF;
    frame->value[38] = (gps->gSpeed >> 0) & 0xFF;
    frame->value[39] = (gps->heading >> 24) & 0xFF;
    frame->value[40] = (gps->heading >> 16) & 0xFF;
    frame->value[41] = (gps->heading >> 8) & 0xFF;
    frame->value[42] = (gps->heading >> 0) & 0xFF;
}
