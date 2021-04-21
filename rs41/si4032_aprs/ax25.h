/*
 *  Original code writen by: Copyright (C) 2013 Lukasz Nidecki SQ5RWU
 *  Modified by tom2238 for usage with libopencm3 and Si4032 FIFO mode, C version
 *
 *  This file is part of ArduinoQAPRS (originaly)
 *
 *  ArduinoQAPRS is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  ArduinoQAPRS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ArduinoQAPRS; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef AX25_RS41_H
#define AX25_RS41_H

#include <stdint.h>

// AX25 header VHF/UHF size in bytes
#define AX25_VHF_HEADER_SIZE 25
// AX25 header HF size in bytes
#define AX25_HF_HEADER_SIZE 45
// AX25 header field byte
#define AX25_HEADER_FIELD_VALUE 0x7E
// AX25 CRC initial value
#define AX25_CRC_INIT_VALUE 0xFFFF
// AX25 CRC polynomial
#define AX25_CRC_POLYNOMIAL 0x8408
// CRC size in bytes
#define AX25_CRC_SIZE 2
// Tone, rates
#define AX25_TONE_MARK 1200
#define AX25_TONE_SPACE 2400
#define AX25_SAMPLE_RATE 13200
// PI constants
#define AX25_WF_TPI 6.2831853f // 2xPI
#define AX25_PI 3.141592654f
#define AX25_TPI AX25_WF_TPI
// AX25 footer flag size
#define AX25_FOOTER_FLAG_SIZE 2
// Si4032 FIFO buffer size
#define AX25_FIFO_BUFFER_SIZE 16
// Maximum APRS frame buffer
#define AX25_TX_BUFFER_SIZE 256

/**
 * @brief Structure defining the basic AX25 frame
 */
typedef struct {
    char to[6]; 			//!< Destination [CALLSIGN]
    uint8_t to_ssid;		//!< SSID destination
    char from[6];			//!< Source [CALLSIGN]
    uint8_t from_ssid;		//!< SSID source
    uint8_t control_field;	//!< Control field Always on 0x03 - UI Frame @see http://www.tapr.org/pub_ax25.html#2.3.4.2
    uint8_t protocolID;		//!< Protocol ID ALWAYS 0xF0 - no layer 3 axles @see http://www.tapr.org/pub_ax25.html#2.2.4
    char packet_content[];	//!< Content of the package. APRS data
} Ax25_BaseFrame;

/**
 * @brief Structure defining the beginning of the AX.25 frame5
 */
typedef struct {
    char to[6]; 			//!< Destination [CALLSIGN]
    uint8_t to_ssid;		//!< SSID destination
    char from[6];			//!< Source [CALLSIGN]
    uint8_t from_ssid;		//!< SSID source
} Ax25_CustomFrameHeader;

int Ax25_Init(char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays);
int Ax25_SendPacketBlocking(const char *buffer, uint16_t length);
int Ax25_SendUIFrameBlocking(char *buffer);

#endif // AX25_RS41_H
