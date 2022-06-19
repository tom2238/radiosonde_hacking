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
#include <libopencm3/stm32/iwdg.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/iwdg.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "ublox6.h"
#include "utils.h"
#include "si4032.h"
#include "frame.h"

// Functions
void usart3_isr(void);
void usart1_isr(void);
static void FrameCalculate(FrameData *frame, uBlox6_GPSData *gps);
static inline void GPSdata_inputRung(void);
static inline void Oscillator_inputRung(void);
static inline void LEDsControl_processRung(void);
static inline void LEDs_outputRung(void);
static inline void Radio_outputRung(void);
// Data
static const char SondeID[8] = {'R','H','1','3','1','2','D','2'};
MCU_IO_state_t mcu_io_state;
// Frame init
// FSK frame
static FrameData dataframe;
// GPS data
static uBlox6_GPSData gpsData;

/**
 * @brief main Simply - It's main
 * @return Never return
 */
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

    // SPI config
    spi_setup();
    // Wait at least 15ms before any initialization SPI commands are sent to the radio
    delay(20);
    // Inicialize Si4032
    Si4032_Init();
    // For some reason we have to do this again
    delay(20);
    Si4032_Init2();

    // Clear watchdog timer
    iwdg_reset();
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(250);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);
    delay(250);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(250);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);
    delay(250);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(250);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);
    delay(250);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    delay(250);

    // Clear watchdog timer
    iwdg_reset();
    // USART3 for serial print
    usart_setup();

    // Initialize Ublox
    // Set 38400 baud rate UART speed
    gps_usart_setup(UBLOX6_UART_SPEED_FAST);
    delay(10);
    // Reset Ublox
    Ublox6_Init(UBLOX6_INIT_RESET);
    // Set 9600 baud rate UART speed
    gps_usart_setup(UBLOX6_UART_SPEED_DEFAULT);
    delay(10);
    // Reset Ublox
    Ublox6_Init(UBLOX6_INIT_RESET);
    // Configure Ublox UART
    Ublox6_Init(UBLOX6_INIT_PROTOCOL);
    // Set 38400 baud rate UART speed
    gps_usart_setup(UBLOX6_UART_SPEED_FAST);
    delay(10);
    // Confgure all other Ublox settings
    Ublox6_Init(UBLOX6_INIT_ALL);

    // Turn off both LEDs
    gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_set(LED_RED_GPIO,LED_RED_PIN);
    delay(500);


    // 62 max bytes for user data
    // 55 bytes for user data
    // Head(8) + User(55) + CRC(2) + ECC(0)
    Frame_Init(62,55,FRAME_MOD_NRZ);

    // Short packet (<= 64), 4800 baud, 2400 Hz deviation, 57 bytes packet size, 80 nibbles
    Si4032_PacketMode(PACKET_TYPE_SHORT,4800,2400,Frame_GetUserLength()+Frame_GetCRCSize(),80);

    console_puts("Init done!\n");
    Ublox6_ClearReceivedMsgCounter();
    mcu_io_state.gpsPulse = FALSE;
    mcu_io_state.led_green = FALSE;
    mcu_io_state.led_red = FALSE;
    mcu_io_state.osc_1hz = FALSE;
	while (1) {
        // 10 ms
        if(mcu_main_pulse_enable == TRUE) {
            // Clear watchdog timer
            iwdg_reset();
            // Clear pulse flag
            mcu_main_pulse_enable = FALSE;
            // Get GPS data
            GPSdata_inputRung();
            // Generate pulses for LEDs
            Oscillator_inputRung();
            // Control LEDs
            LEDsControl_processRung();
            // Write LEDs
            LEDs_outputRung();
            // Write packet
            Radio_outputRung();
        }

	}

	return 0;
}

/**
 * @brief usart3_isr XDATA USART interrupt routine
 */
void usart3_isr(void) {
    /* Check if we were called because of RXNE. */
    if (((USART_CR1(XDATA_USART) & USART_CR1_RXNEIE) != 0) && ((USART_SR(XDATA_USART) & USART_SR_RXNE) != 0)) {
        /*uint16_t c = usart_recv(XDATA_USART);
        if (c == 'C') {
            gpio_clear(LED_RED_GPIO,LED_RED_PIN);
        } else if (c == 'S'){
            gpio_set(LED_RED_GPIO,LED_RED_PIN);
        }*/
    }
}

/**
 * @brief usart1_isr GPS USART interrupt routine
 */
void usart1_isr(void) {
    /* Check if we were called because of RXNE. */
    if (((USART_CR1(GPS_USART) & USART_CR1_RXNEIE) != 0) && ((USART_SR(GPS_USART) & USART_SR_RXNE) != 0)) {
        uint16_t c = usart_recv(GPS_USART);
        //console_putc(c); // Debug info
        Ublox6_HandleByte((uint8_t)c);
    }
}

/**
 * @brief FrameCalculate Fill user data (with user length) into frame
 * @param frame TX frame block
 * @param gps Current GPS data
 */
static void FrameCalculate(FrameData *frame, uBlox6_GPSData *gps) {
    // Frame user length is 55 bytes
    static uint16_t frame_cnt = 0;
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
    // GPS NAV SOL
    frame->value[43] = (gps->iTOW >> 24) & 0xFF;
    frame->value[44] = (gps->iTOW >> 16) & 0xFF;
    frame->value[45] = (gps->iTOW >> 8) & 0xFF;
    frame->value[46] = (gps->iTOW >> 0) & 0xFF;
    frame->value[47] = (gps->week >> 8) & 0xFF;
    frame->value[48] = (gps->week >> 0) & 0xFF;
    frame->value[49] = (gps->pDOP >> 24) & 0xFF;
    frame->value[50] = (gps->pDOP >> 16) & 0xFF;
    frame->value[51] = (gps->pDOP >> 8) & 0xFF;
    frame->value[52] = (gps->pDOP >> 0) & 0xFF;
    // Frame count
    frame->value[53] = (frame_cnt >> 8) & 0xFF;
    frame->value[54] = (frame_cnt >> 0) & 0xFF;
    // Sonde ID
    frame->value[55] = SondeID[0];
    frame->value[56] = SondeID[1];
    frame->value[57] = SondeID[2];
    frame->value[58] = SondeID[3];
    frame->value[59] = SondeID[4];
    frame->value[60] = SondeID[5];
    frame->value[61] = SondeID[6];
    frame->value[62] = SondeID[7];
    frame_cnt++;
}

/**
 * @brief GPSdata_inputRung GPS data receive
 */
static inline void GPSdata_inputRung(void) {
    // Check GPS lock
    if(Ublox6_GetReceivedMsgCounter() >= 4) {
        // Get current GPS data
        Ublox6_GetLastData(&gpsData);
        // Clear message counter
        Ublox6_ClearReceivedMsgCounter();
        // Set GPS pulse
        mcu_io_state.gpsPulse = TRUE;
    } else {
        mcu_io_state.gpsPulse = FALSE;
    }
}

/**
 * @brief Oscillator_inputRung
 */
static inline void Oscillator_inputRung(void) {
    static uint8_t timer = 0;
    timer++;
    if(timer >= 100) {
        mcu_io_state.osc_1hz = TRUE;
        timer = 0;
    } else {
        mcu_io_state.osc_1hz = FALSE;
    }
}

/**
 * @brief LEDsControl_processRung Set LEDs states based on GPS data
 */
static inline void LEDsControl_processRung(void) {
    // 3D fix
    if(gpsData.gpsFix == UBLOX6_GPSFIX_3D_FIX) {
        // Set Green LED if GPS is fixed
        mcu_io_state.led_green = TRUE;
    } else  {
        // If not blinking with Green LED
        if(mcu_io_state.osc_1hz == TRUE) {
            mcu_io_state.led_green = !mcu_io_state.led_green;
        }
    }
    // If is valid GPS week number and GPS time of week

    if((gpsData.navSolFlags & 0x0C) == 0x0C) {
        // Turn off Red led if time is fixed
        mcu_io_state.led_red = FALSE;
    } else {
        // If not blinking with Red LED
        if(mcu_io_state.osc_1hz == TRUE) {
            mcu_io_state.led_red = !mcu_io_state.led_red;
        }
    }
}

/**
 * @brief LEDs_outputRung Set LEDs outputs
 */
static inline void LEDs_outputRung(void) {
    // Green
    if(mcu_io_state.led_green) {
        gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    } else {
        gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);
    }
    // Red
    if(mcu_io_state.led_red) {
        gpio_clear(LED_RED_GPIO,LED_RED_PIN);
    } else {
        gpio_set(LED_RED_GPIO,LED_RED_PIN);
    }
}

/**
 * @brief Radio_outputRung Send TX data over radio
 */
static inline void Radio_outputRung(void) {
    if(mcu_io_state.gpsPulse == TRUE) {
        // New packet
        dataframe = Frame_NewData(Frame_GetUserLength() + Frame_GetHeadSize() + Frame_GetECCSize() + Frame_GetCRCSize(), Frame_GetCoding());
        // Calculate new frame data
        FrameCalculate(&dataframe,&gpsData);
        // Calculate CRC16(2)
        Frame_CalculateCRC16(&dataframe);
        Frame_XOR(&dataframe,0); // XORing NRZ frame
        // Preamble(40) and header(8) is added in Si4032
        Si4032_WriteShortPacket((((uint8_t*)&dataframe.value) + Frame_GetHeadSize()), Frame_GetUserLength()+Frame_GetCRCSize());
    }
}
