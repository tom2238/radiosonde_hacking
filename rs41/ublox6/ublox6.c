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

// Global LibOpenCM3 libraries
#include <libopencm3/stm32/usart.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/usart.h>
// Another libraries
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "ublox6.h"
#include "init.h"
#include "utils.h"

// Private
static void Ublox6_CalculateChecksum(uBlox6_Checksum *checksum, uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t size);
static void Ublox6_SendPayload(uint8_t msgClass, uint8_t msgID, uint8_t *payload, uint16_t size);
static void Ublox6_SendConfigRST(uBlox6_CFGRST_Payload *message);
static void Ublox6_SendConfigPRT(uBlox6_CFGPRT_Payload *message);
static void Ublox6_SendConfigRXM(ublox6_CFGRXM_Payload *message);
static void Ublox6_SendConfigMSG(uBlox6_CFGMSG_Payload *message);
static void Ublox6_SendConfigNAV5(uBlox6_CFGNAV5_Payload *message);
static void Ublox6_HandlePacket(uBlox6_Packet *packet);
static uint8_t Ublox6_WaitForACK(void);

// Ack/Nack Messages: i.e. as replies to CFG Input Messages
static volatile uint8_t _ack_received = 0;
static volatile uint8_t _nack_received = 0;

// Current GPS data
static uBlox6_GPSData _current_GPSData;

/**
 * @brief Ublox6_CalculateChecksum
 * @param checksum Checksum structure
 * @param msgClass UBX Message class
 * @param msgId UBX Message ID
 * @param payload Command payload content
 * @param size Size of payload
 */
static void Ublox6_CalculateChecksum(uBlox6_Checksum *checksum, uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t size) {
    // The checksum algorithm used is the 8-Bit Fletcher Algorithm, which is used in the TCP standard (RFC 1145).
    // Initial values
    checksum->ck_a = 0x00;
    checksum->ck_b = 0x00;
    // Calculate message class and message ID
    checksum->ck_a += msgClass;
    checksum->ck_b += checksum->ck_a;
    checksum->ck_a += msgId;
    checksum->ck_b += checksum->ck_a;
    // Calculate size of payload
    checksum->ck_a += size & 0xFF;
    checksum->ck_b += checksum->ck_a;
    checksum->ck_a += size >> 8;
    checksum->ck_b += checksum->ck_a;
    // Calculate payload content
    uint16_t i;
    for(i=0;i<size;i++) {
        checksum->ck_a += payload[i];
        checksum->ck_b += checksum->ck_a;
    }
}

/**
 * @brief Ublox6_SendPayload
 * @param msgClass
 * @param msgID
 * @param payload
 * @param size
 */
static void Ublox6_SendPayload(uint8_t msgClass, uint8_t msgID, uint8_t *payload, uint16_t size) {
    uBlox6_Checksum crc;
    Ublox6_CalculateChecksum(&crc,msgClass,msgID,payload,size);
    // Send header
    usart_send_blocking(GPS_USART,UBLOX6_UBX_SYNC_CH1);
    usart_send_blocking(GPS_USART,UBLOX6_UBX_SYNC_CH2);
    // Send message class and ID
    usart_send_blocking(GPS_USART,msgClass);
    usart_send_blocking(GPS_USART,msgID);
    // Send payload size
    usart_send_blocking(GPS_USART,(uint8_t)(size & 0xFF));
    usart_send_blocking(GPS_USART,(uint8_t)(size >> 8));
    // Send payload content
    uint16_t i;
    for(i=0;i<size;i++) {
        usart_send_blocking(GPS_USART,payload[i]);
    }
    // Send checksum
    usart_send_blocking(GPS_USART,crc.ck_a);
    usart_send_blocking(GPS_USART,crc.ck_b);
}

/**
 * @brief Ublox6_SendConfigRST
 * @param message
 */
static void Ublox6_SendConfigRST(uBlox6_CFGRST_Payload *message) {
    Ublox6_SendPayload(UBLOX6_CLASS_ID_CFG,UBLOX6_MSG_ID_CFGRST,(uint8_t*)message,sizeof(uBlox6_CFGRST_Payload));
}

/**
 * @brief Ublox6_SendConfigPRT
 * @param message
 */
static void Ublox6_SendConfigPRT(uBlox6_CFGPRT_Payload *message) {
    Ublox6_SendPayload(UBLOX6_CLASS_ID_CFG,UBLOX6_MSG_ID_CFGPRT,(uint8_t*)message,sizeof(uBlox6_CFGPRT_Payload));
}

/**
 * @brief Ublox6_SendConfigRXM
 * @param message
 */
static void Ublox6_SendConfigRXM(ublox6_CFGRXM_Payload *message) {
    Ublox6_SendPayload(UBLOX6_CLASS_ID_CFG,UBLOX6_MSG_ID_CFGRXM,(uint8_t*)message,sizeof(ublox6_CFGRXM_Payload));
}

/**
 * @brief Ublox6_SendConfigMSG
 * @param message
 */
static void Ublox6_SendConfigMSG(uBlox6_CFGMSG_Payload *message) {
    Ublox6_SendPayload(UBLOX6_CLASS_ID_CFG,UBLOX6_MSG_ID_CFGMSG,(uint8_t*)message,sizeof(uBlox6_CFGMSG_Payload));
}

/**
 * @brief Ublox6_SendConfigNAV5
 * @param message
 */
static void Ublox6_SendConfigNAV5(uBlox6_CFGNAV5_Payload *message) {
    Ublox6_SendPayload(UBLOX6_CLASS_ID_CFG,UBLOX6_MSG_ID_CFGNAV5,(uint8_t*)message,sizeof(uBlox6_CFGNAV5_Payload));
}

/**
 * @brief Ublox6_Init
 */
void Ublox6_Init(void) {
    uBlox6_CFGRST_Payload cfgrst;       // Reset
    cfgrst.navBbrMask = 0xFFFF;         // 0xFFFF Coldstart
    cfgrst.resetMode = 0x01;            // 0x01 - Controlled Software reset
    cfgrst.reserved1 = 0x00;            // Reserved
    Ublox6_SendConfigRST(&cfgrst);
    delay(200);

    uBlox6_CFGPRT_Payload cfgprt;       // Preferred protocol(s) needs to be enabled on a port
    cfgprt.portID = 1;                  // Port Identifier Number,  1 = UART1
    cfgprt.reserved0 = 0;               // Reserved
    cfgprt.txReady = 0;                 // TX ready PIN configuration, TX ready feature disabled
    cfgprt.mode = 0b00100011000000;     // A bit mask describing the UART mode, Character Length = 8bit, No Parity, Number of Stop Bits = 1 Stop Bit
    cfgprt.baudRate = UBLOX6_UART_SPEED_FAST;   // 38400 Bit/s
    cfgprt.inProtoMask = 1;             // A mask describing which input protocols are active. 1 = UBX enable
    cfgprt.outProtoMask = 1;            // A mask describing which output protocols are active. 1 = UBX enable
    cfgprt.reserved4 = 0;               // Always set to zero
    cfgprt.reserved5 = 0;               // Always set to zero
    Ublox6_SendConfigPRT(&cfgprt);
    delay(10);
    /*
     *  Change USART baudrate
     */
    gps_usart_setup(UBLOX6_UART_SPEED_FAST);
    delay(100);
    ublox6_CFGRXM_Payload cfgrxm;
    cfgrxm.lpMode = 4;                  // Low Power Mode, 4 = Continuous Mode
    cfgrxm.reserved1 = 8;               // Always set to 8
    do {
        Ublox6_SendConfigRXM(&cfgrxm);
    } while (!Ublox6_WaitForACK());

    uBlox6_CFGMSG_Payload cfgmsg;           // Activate certain messages on each port
    cfgmsg.msgClass = UBLOX6_CLASS_ID_NAV;  // Message Class
    cfgmsg.msgID = UBLOX6_MSG_ID_NAVPOSLLH; // Message Identifier, NAV-POSLLH
    cfgmsg.rate = 1;                        // Send rate on current Target
    do {
        Ublox6_SendConfigMSG(&cfgmsg);
    } while (!Ublox6_WaitForACK());
    cfgmsg.msgClass = UBLOX6_CLASS_ID_NAV;  // Message Class
    cfgmsg.msgID = UBLOX6_MSG_ID_NAVSOL;    // Message Identifier, NAV-SOL
    cfgmsg.rate = 1;                        // Send rate on current Target
    do {
        Ublox6_SendConfigMSG(&cfgmsg);
    } while (!Ublox6_WaitForACK());
    cfgmsg.msgClass = UBLOX6_CLASS_ID_NAV;  // Message Class
    cfgmsg.msgID = UBLOX6_MSG_ID_NAVTIMEUTC;// Message Identifier, NAV-TIMEUTC
    cfgmsg.rate = 1;                        // Send rate on current Target
    do {
        Ublox6_SendConfigMSG(&cfgmsg);
    } while (!Ublox6_WaitForACK());
    cfgmsg.msgClass = UBLOX6_CLASS_ID_NAV;  // Message Class
    cfgmsg.msgID = UBLOX6_MSG_ID_NAVVELNED; // Message Identifier, NAV-VELNED
    cfgmsg.rate = 1;                        // Send rate on current Target
    do {
        Ublox6_SendConfigMSG(&cfgmsg);
    } while (!Ublox6_WaitForACK());

    uBlox6_CFGNAV5_Payload cfgnav5;     // Navigation Engine Settings
    cfgnav5.mask = 0b00000001111111111; // Apply dynamic model settings, Apply minimum elevation settings, Apply fix mode settings, Apply position mask settings,  Apply time mask settings, Apply static hold settings, Apply DGPS settings.
    cfgnav5.dynModel = 7;               // Dynamic Platform model, 7 = Airborne with <2g Acceleration, 0 = Portable
    cfgnav5.fixMode = 2;                // Position Fixing Mode, 3 = Fix 3D only
    cfgnav5.fixedAlt = 0;               // Fixed altitude (mean sea level) for 2D fix mode, 0*0.01 meters
    cfgnav5.fixedAltVar = 10000;        // Fixed altitude variance for 2D mode. 10000*0.0001 meters^2
    cfgnav5.minElev = 5;                // Minimum Elevation for a GNSS satellite to be used in NAV, 5 deg
    cfgnav5.drLimit = 0;                // Reserved
    cfgnav5.pDop = 25;                  // Position DOP Mask to use, 0.1*25
    cfgnav5.tDop = 25;                  // Time DOP Mask to use, 0.1*25
    cfgnav5.pAcc = 100;                 // Position Accuracy Mask, 100 meters
    cfgnav5.tAcc = 300;                 // Time Accuracy Mask, 300 meters
    cfgnav5.staticHoldThresh = 0;       // Static hold threshold, 0 cm/s
    cfgnav5.dgpsTimeOut = 2;            // DGPS timeout. 2 seconds
    cfgnav5.cnoThreshNumSVs = 0;        // Number of satellites required to have C/N0 above cnoThresh for a valid fix. 0
    cfgnav5.cnoThresh = 0;              // C/N0 threshold for a valid fix. 0 dBHz
    cfgnav5.reserved2 = 0;              // Always set to zero
    cfgnav5.reserved3 = 0;              // Always set to zero
    cfgnav5.reserved4 = 0;              // Always set to zero
    do {
        Ublox6_SendConfigNAV5(&cfgnav5);
    } while (!Ublox6_WaitForACK());
}

/**
 * @brief Ublox6_HandleByte
 * @param data
 */
void Ublox6_HandleByte(uint8_t data) {
    static uint8_t sync = 0;
    static uint8_t buffer_pos = 0;
    static uint8_t incoming_packet_buffer[sizeof(uBlox6_Packet) + sizeof(uBlox6_Checksum)];
    static uBlox6_Packet *incoming_packet = (uBlox6_Packet *)incoming_packet_buffer;
    // Find 0xB5 0x62 header
    if(!sync) {
        if(!buffer_pos && data == UBLOX6_UBX_SYNC_CH1) {
            buffer_pos = 1;
            incoming_packet->header.sc1 = data;
        } else if(buffer_pos == 1 && data == UBLOX6_UBX_SYNC_CH2) {
            sync = 1;
            buffer_pos = 2;
            incoming_packet->header.sc2 = data;
        } else {
            buffer_pos = 0;
        }
    } else { // Header found
        ((uint8_t *)incoming_packet)[buffer_pos] = data;
        if (((size_t)buffer_pos >= sizeof(uBlox6_Header)-1) && ((size_t)buffer_pos-1 == ((size_t)incoming_packet->header.payloadSize + sizeof(uBlox6_Header) + sizeof(uBlox6_Checksum)))){
            Ublox6_HandlePacket((uBlox6_Packet *)incoming_packet);
            buffer_pos = 0;
            sync = 0;
        } else {
            buffer_pos++;
            if (buffer_pos >= sizeof(uBlox6_Packet) + sizeof(uBlox6_Checksum)) {
                buffer_pos = 0;
                sync = 0;
            }
        }
    }
}

/**
 * @brief Ublox6_HandlePacket
 * @param packet
 */
static void Ublox6_HandlePacket(uBlox6_Packet *packet) {
    // Check checksum for received packet
    uBlox6_Checksum ck_calculated;
    Ublox6_CalculateChecksum(&ck_calculated,packet->header.messageClass,packet->header.messageId,(const uint8_t *)&packet->data,packet->header.payloadSize);
    // Get checksum from received packet
    uBlox6_Checksum *ck_received = (uBlox6_Checksum *)(((uint8_t*)&packet->data) + packet->header.payloadSize);
    // Check if checksums is same
    if (ck_calculated.ck_a == ck_received->ck_a && ck_calculated.ck_b == ck_received->ck_b) {
        // If yes, check Class and ID to get correct data
        if (packet->header.messageClass == UBLOX6_CLASS_ID_ACK && packet->header.messageId == UBLOX6_MSG_ID_ACKACK) {
            _ack_received = 1;
        } else if (packet->header.messageClass == UBLOX6_CLASS_ID_ACK && packet->header.messageId == UBLOX6_MSG_ID_ACKNAK) {
            _nack_received = 1;
        } else if (packet->header.messageClass == UBLOX6_CLASS_ID_NAV && packet->header.messageId == UBLOX6_MSG_ID_NAVTIMEUTC) {
            _current_GPSData.day = packet->data.navtimeutc.day;
            _current_GPSData.hour = packet->data.navtimeutc.hour;
            _current_GPSData.min = packet->data.navtimeutc.min;
            _current_GPSData.month = packet->data.navtimeutc.month;
            _current_GPSData.sec = packet->data.navtimeutc.sec;
            _current_GPSData.year = packet->data.navtimeutc.year;
        } else if (packet->header.messageClass == UBLOX6_CLASS_ID_NAV && packet->header.messageId == UBLOX6_MSG_ID_NAVSOL) {
            _current_GPSData.gpsFix = packet->data.navsol.gpsFix;
            _current_GPSData.numSV = packet->data.navsol.numSV;
        } else if (packet->header.messageClass == UBLOX6_CLASS_ID_NAV && packet->header.messageId == UBLOX6_MSG_ID_NAVPOSLLH) {
            _current_GPSData.lat = packet->data.navposllh.lat;
            _current_GPSData.lon = packet->data.navposllh.lon;
            _current_GPSData.hMSL = packet->data.navposllh.hMSL;
        } else if (packet->header.messageClass == UBLOX6_CLASS_ID_NAV && packet->header.messageId == UBLOX6_MSG_ID_NAVVELNED) {
            _current_GPSData.gSpeed = packet->data.navvelned.gSpeed;
            _current_GPSData.heading = packet->data.navvelned.heading;
            _current_GPSData.speed = packet->data.navvelned.speed;
        }
    }
}

/**
 * @brief Ublox6_WaitForACK
 * @return
 */
static uint8_t Ublox6_WaitForACK(void) {
    _ack_received = 0;
    _nack_received = 0;
    uint8_t timeout = 200;
    while(!_ack_received && !_nack_received){
        delay(1);
        if (!timeout--){
            break;
        }
    }
    return _ack_received;
}

/**
 * @brief ublox_get_last_data
 * @param gpsEntry
 */
void Ublox6_GetLastData(uBlox6_GPSData *gpsEntry) {
  //usart_disable_rx_interrupt(GPS_USART);
  memcpy(gpsEntry, &_current_GPSData, sizeof(uBlox6_GPSData));
  //usart_enable_rx_interrupt(GPS_USART);
}
