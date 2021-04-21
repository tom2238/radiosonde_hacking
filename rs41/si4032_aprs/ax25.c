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

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "ax25.h"
#include "si4032.h"
#include "utils.h"

// Private variables
// Packet CRC
static uint16_t _Ax25_packet_CRC;
// AFSK1200 phase
static float _sine_phase = 0;
// NRZI tone
static uint16_t _Ax25_CurrentTone = AX25_TONE_MARK;
// APRS frame data
static uint8_t _tmpData[255];
// Src, Dst address, AX25
static char _from_addr[6];
static uint8_t _from_ssid;
static char _to_addr[6];
static uint8_t _to_ssid;
static char* _relays[3*7];
// Only at begin of frame
static uint8_t _enable_packetTX = 0;
// Bytes counter for FIFO loader
static uint8_t _fifo_byte_loader_cnt;

// Private functions
static void Ax25_SendHeader(void);
static void Ax25_SendFooter(void);
static void Ax25_SendByte(uint8_t byte);
static void Ax25_WriteRawBit(uint16_t tone);
static float Ax25_Sin(float x);
static void Ax25_AddBitToFIFO(uint8_t bit);
static inline void Ax25_WriteToneMark(void);
static inline void Ax25_WriteToneSpace(void);
static inline void Ax25_ToggleTone(void);
static inline void Ax25_WriteCurrentTone(void);
static void Ax25_CalcCRC(uint8_t bit);
static void Ax25_SetFromAddress(char* from_addr, uint8_t from_ssid);
static void Ax25_SetToAddress(char* to_addr, uint8_t to_ssid);
static void Ax25_SetRelays(char *relays);
static void Ax25_ParseRelays(const char* relays, char* dst);
static int Ax25_Send(char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays, char* packet_content);
static uint16_t Ax25_Bits2Byte(uint8_t bits[]);

int Ax25_Init(char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays) {
    Ax25_SetFromAddress(from_addr,from_ssid);
    Ax25_SetToAddress(to_addr,to_ssid);
    Ax25_SetRelays(relays);
    return 0;
}

int Ax25_SendPacketBlocking(const char *buffer, uint16_t length) {
    // Enable packet config once
    _enable_packetTX = 0;
    // Reset AFSK1200 carrier phase
    _sine_phase = -AX25_PI;
    // Set initial tone to MARK
    _Ax25_CurrentTone = AX25_TONE_MARK;
    // Loading bytes from zero
    _fifo_byte_loader_cnt = 0;
    Ax25_SendHeader();
    for (size_t i = 0; i < length; i++) {
        Ax25_SendByte((uint8_t)(buffer[i]));
    }
    Ax25_SendFooter();
    // Enable packet config once
    _enable_packetTX = 0;
    // Wait for send
    while(!Si4032_IsPacketSent());
    // Disable transmitter
    Si4032_DisableTx();
    return 0;
}

int Ax25_SendDataBlocking(char *buffer) {
    return Ax25_Send((char*)_from_addr,_from_ssid,(char*)_to_addr,_to_ssid,(char*)_relays,buffer);
}


static void Ax25_SendHeader(void) {
    _Ax25_packet_CRC = AX25_CRC_INIT_VALUE;
    uint16_t i;
    for(i=0;i<AX25_VHF_HEADER_SIZE;i++) {
        Ax25_SendByte(AX25_HEADER_FIELD_VALUE);
    }
}

/**
 * 16-bit CRC-CCITT
 * MBS + LE!!!
 * @see: http://www.tapr.org/pub_ax25.html#2.2.7
 * @see: http://www.tapr.org/pub_ax25.html#2.2.8
 * @brief Ax25_SendFooter
 */
static void Ax25_SendFooter(void) {
    static uint8_t tmp_byte;
    tmp_byte = (uint8_t)((_Ax25_packet_CRC >> 8) ^ 0xFF);
    Ax25_SendByte((uint8_t)((_Ax25_packet_CRC)^0xFF));
    Ax25_SendByte(tmp_byte);
    Ax25_SendByte(AX25_HEADER_FIELD_VALUE);
    Ax25_SendByte(AX25_HEADER_FIELD_VALUE);
    Ax25_SendByte(AX25_HEADER_FIELD_VALUE);
}

static void Ax25_SendByte(uint8_t byte) {
    static uint8_t i, lsb_bit, is_flag, bit_stuffing_counter=0;
    // Check if byte is not a flag, because we give it in a special way
    is_flag = (uint8_t)(byte == AX25_HEADER_FIELD_VALUE);
    for(i=0;i<8;i++) {
        // Save LSB bit
        lsb_bit = (uint8_t)(byte & 0x01);

        if(is_flag) {
            bit_stuffing_counter = 0;
        } else {
            Ax25_CalcCRC(lsb_bit);
        }

        // NRZI code, No Return to Zero Inverted
        if(lsb_bit) { // Sending 1
            bit_stuffing_counter++;
            //Ax25_WriteToneMark();
            if(bit_stuffing_counter == 5) {
                // After 5 bits in hight insert low bit
                Ax25_WriteCurrentTone();
                Ax25_ToggleTone();
                bit_stuffing_counter = 0;
            }
        } else { // Sending 0
            bit_stuffing_counter = 0;
            Ax25_ToggleTone();
        }
        Ax25_WriteCurrentTone();
        // Shift byte right
        byte >>= 1;
    }
}

static void Ax25_WriteRawBit(uint16_t tone) {
    // Samples per tone
    unsigned int tone_samples_per_bit = AX25_SAMPLE_RATE/AX25_TONE_MARK;
    // Delta phase of sine carrier, depending on tone frequency
    float deltaph = AX25_WF_TPI*tone/AX25_SAMPLE_RATE;
    int16_t sig;

    while(tone_samples_per_bit--) {
        // Calculate AFSK1200 carrier value
        sig = 256 * Ax25_Sin(_sine_phase);
        if(sig >= 0) {
            Ax25_AddBitToFIFO(1);
        } else {
            Ax25_AddBitToFIFO(0);
        }
        // Add phase for new sine calculation
        _sine_phase += deltaph;
        // If is phase out of range, change phase to begin
        if(_sine_phase > AX25_PI) {
            _sine_phase =  - (AX25_PI);
        }
    }
}

/**
 * @brief Ax25_Sin Minimal fast sine calculation, parabola aproximation
 * @param x Angle value in radians from -PI to +PI
 * @return Calculated sine value
 */
static float Ax25_Sin(float x) {
    const float B = 4 / AX25_PI;
    const float C = -4 / (AX25_PI*AX25_PI);

    return -(B * x + C * x * ((x < 0) ? -x : x));
}

static void Ax25_AddBitToFIFO(uint8_t bit) {
    static uint8_t new_bit_buff[8];
    static uint8_t new_bit_cnt = 0;
    static uint8_t new_byte;
    static uint8_t new_byte_buff[32];
    // Add bits into byte array
    new_bit_buff[new_bit_cnt] = bit;
    new_bit_cnt++;
    // If one byte
    if(new_bit_cnt == 8) {
        new_bit_cnt = 0;
        // Convert into byte
        new_byte = Ax25_Bits2Byte(new_bit_buff);
        // Add into 32 byte array
        new_byte_buff[_fifo_byte_loader_cnt] = new_byte;
        _fifo_byte_loader_cnt++;
        if(_fifo_byte_loader_cnt == 32) {
            if(_enable_packetTX == 0) {
                // Disable beginning config
                _enable_packetTX = 1;
                // Clear TX FIFO on start
                Si4032_ClearFIFO();
                // Write 32 bytes into FIFO
                Si4032_WritePacketData(new_byte_buff,0,32);
                // Enable packet transmission
                Si4032_PacketTx();
                // Clear byte counter
                _fifo_byte_loader_cnt = 0;
            } else {
                // Wait for empty FIFO
                while(!Si4032_IsFIFOEmpty());
                // Write 32 bytes into FIFO
                Si4032_WritePacketData(new_byte_buff,0,32);
                // Clear byte counter
                _fifo_byte_loader_cnt = 0;
            }
        }
    }
}

static inline void Ax25_WriteToneMark(void) {
    Ax25_WriteRawBit(AX25_TONE_MARK);
}

static inline void Ax25_WriteToneSpace(void) {
    Ax25_WriteRawBit(AX25_TONE_SPACE);
}

static inline void Ax25_ToggleTone(void) {
    _Ax25_CurrentTone = (_Ax25_CurrentTone == AX25_TONE_SPACE) ? AX25_TONE_MARK : AX25_TONE_SPACE;
}

static inline void Ax25_WriteCurrentTone(void) {
    Ax25_WriteRawBit(_Ax25_CurrentTone);
}

static void Ax25_CalcCRC(uint8_t bit) {
    static unsigned short crc_tmp;
    // XOR LSB bit of CRC with the latest bit
    crc_tmp = _Ax25_packet_CRC ^ bit;
    // Shift 16-bit CRC one bit to the right
    _Ax25_packet_CRC >>= 1;
    // If XOR result from above has LSB bit set
    if(crc_tmp & 0x0001) {
        // XOR CRC value with polynomial constant
        _Ax25_packet_CRC ^= AX25_CRC_POLYNOMIAL;
    }
}

static void Ax25_SetFromAddress(char* from_addr, uint8_t from_ssid) {
    memset(_from_addr,' ',sizeof(_from_addr));
    strncpy(_from_addr,from_addr,sizeof(_from_addr));
    _from_ssid = from_ssid;
}

static void Ax25_SetToAddress(char* to_addr, uint8_t to_ssid) {
    memset(_to_addr,' ',sizeof(_to_addr));
    strncpy(_to_addr,to_addr,sizeof(_to_addr));
    _to_ssid = to_ssid;
}

static void Ax25_SetRelays(char *relays) {
    Ax25_ParseRelays(relays, (char*)_relays);
}

static void Ax25_ParseRelays(const char* relays, char* dst) {
    uint8_t relays_len = (uint8_t) strlen(relays);
    uint8_t relays_ptr = 0;
    uint8_t dst_ptr = 0;
    uint8_t fill_length = 0;
    for(relays_ptr=0;relays_ptr<relays_len;relays_ptr++) {
        if(relays[relays_ptr] == ',' || relays_ptr == relays_len-1) {
            if(relays[relays_ptr != ',']) {
                dst[dst_ptr] = relays[relays_ptr] == '-' ? ' ' : relays[relays_ptr]; // change ',' to ' '
                dst_ptr++;
            }
            // End of element
            if(dst_ptr<7) {
                fill_length = (uint8_t)(7 - dst_ptr);
            } else if (dst_ptr > 7 && dst_ptr < 7+7) {
                fill_length = (uint8_t)(7 + 7 - dst_ptr);
            } else if (dst_ptr > 7+7 && dst_ptr < 7+7+7) {
                fill_length = (uint8_t)(7 + 7 + 7 - dst_ptr);
            }
            while (fill_length) {
                dst[dst_ptr] = ' ';
                fill_length--;
                dst_ptr++;
            }
        } else {
            dst[dst_ptr] = relays[relays_ptr] == '-' ? ' ' : relays[relays_ptr]; // change ',' to ' '
            dst_ptr++;
        }
    }
    dst[dst_ptr] = 0;
}

static int Ax25_Send(char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays, char* packet_content) {
    Ax25_CustomFrameHeader *bf;
    bf = (Ax25_CustomFrameHeader*)_tmpData;
    memset(bf->from,' ',sizeof(bf->from));
    strncpy(bf->from, from_addr,sizeof(bf->from));
    memset(bf->to, ' ', sizeof(bf->to));
    strncpy(bf->to, to_addr, sizeof(bf->to));
    bf->to_ssid = to_ssid;
    bf->from_ssid = (uint8_t) (from_ssid > '@' ? from_ssid - 6 : from_ssid);
    uint8_t relay_size = (uint8_t) strlen(relays);
    strcpy((char*)(_tmpData+sizeof(Ax25_CustomFrameHeader)), relays);
    uint8_t i;
    for(i=0;i<sizeof(Ax25_CustomFrameHeader)+relay_size;i++){
        _tmpData[i] = (_tmpData[i]) << 1;
    }
    // The last bit in addresses must be set to 1
    _tmpData[sizeof(Ax25_CustomFrameHeader)+relay_size - 1] |= 1;
    // control_field
    _tmpData[(sizeof(Ax25_CustomFrameHeader)+relay_size)] = 0x03;
    // protocolID
    _tmpData[(sizeof(Ax25_CustomFrameHeader)+relay_size+1)] = 0xf0;

    strncpy((char*)(_tmpData+sizeof(Ax25_CustomFrameHeader)+relay_size+2), packet_content, strlen(packet_content));
    return Ax25_SendPacketBlocking((char*)_tmpData, (sizeof(Ax25_CustomFrameHeader)+relay_size+2+strlen(packet_content)));
}

static uint16_t Ax25_Bits2Byte(uint8_t bits[]) {
    int i, byteval=0, d=1;
    for (i = 0; i < 8; i++) {     // little endian
        /* for (i = 7; i >= 0; i--) { // big endian */
        if (bits[i] == 1)  {
            byteval += d;
        }
        else if (bits[i] == 0)  {
            byteval += 0;
        }
        else {
            return 0x0100;
        }
        d <<= 1;
    }
    return byteval;
}
