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

/*
 * u-blox 6 GPS/GLONASS/QZSS Receiver Description
 * Including Protocol Specification V14
 * GPS.G6-SW-12013, 60119,   5 Jul 2012
 * https://www.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf
 */

#ifndef UBLOX6_H
#define UBLOX6_H

// UBX Class IDs
#define UBLOX6_CLASS_ID_NAV 0x01
#define UBLOX6_CLASS_ID_RXM 0x02
#define UBLOX6_CLASS_ID_INF 0x04
#define UBLOX6_CLASS_ID_ACK 0x05
#define UBLOX6_CLASS_ID_CFG 0x06
#define UBLOX6_CLASS_ID_MON 0x0A
#define UBLOX6_CLASS_ID_AID 0x0B
#define UBLOX6_CLASS_ID_TIM 0x0D
#define UBLOX6_CLASS_ID_ESF 0x10

// Message Acknowledged
#define UBLOX6_MSG_ID_ACKACK 0x01
// Message Not-Acknowledged
#define UBLOX6_MSG_ID_ACKNAK 0x00

// UBX protocol header
#define UBLOX6_UBX_SYNC_CH1 0xB5
#define UBLOX6_UBX_SYNC_CH2 0x62

// Vaisala RS41 GPS UART
#define GPS_USART USART1
#define GPS_USART_GPIO GPIOA
#define GPS_USART_RCC_GPIO RCC_GPIOA
#define GPS_USART_RCC RCC_USART1

// UART1 speed
#define UBLOX6_UART_SPEED_DEFAULT 9600
#define UBLOX6_UART_SPEED_FAST 38400

// Get/Set Port Configuration for UART
#define UBLOX6_MSG_ID_CFGPRT 0x00
typedef struct {
    uint8_t portID;         // Port Identifier Number (= 1 or 2 for UART ports) [- -]
    uint8_t reserved0;      // Reserved [- -]
    uint16_t txReady;       // Reserved (Alwyas set to zero) up to Firmware 7.01,TX ready PIN configuration (since Firmware 7.01)
    uint32_t mode;          // A bit mask describing the UART mode [- -]
    uint32_t baudRate;      // Baudrate in bits/second [- Bit/s]
    uint16_t inProtoMask;   // A mask describing which input protocols areactive. Each bit of this mask is used for a protocol. Through that, multiple protocols can be definedon a single port. [- -]
    uint16_t outProtoMask;  // A mask describing which output protocols areactive. Each bit of this mask is used for a protocol. Through that, multiple protocols can be definedon a single port. [- -]
    uint16_t reserved4;     // Always set to zero [- -]
    uint16_t reserved5;     // Always set to zero [- -]
} uBlox6_CFGPRT_Payload;

// Reset Receiver / Clear Backup Data Structures
#define UBLOX6_MSG_ID_CFGRST 0x04
typedef struct {
    uint16_t navBbrMask;    // BBR Sections to clear. The following Special Sets apply: [- -]
                            // 0x0000 Hotstart
                            // 0x0001 Warmstart
                            // 0xFFFF Coldstart
    uint8_t resetMode;		// Reset Type [- -]
                            // - 0x00 - Hardware reset (Watchdog) immediately
                            // - 0x01 - Controlled Software reset
                            // - 0x02 - Controlled Software reset (GPS only)
                            // - 0x04 - Hardware reset (Watchdog) after shutdown (>=FW6.0)
                            // - 0x08 - Controlled GPS stop
                            // - 0x09 - Controlled GPS start
                            // - 0x09 - Controlled GPS start
    uint8_t reserved1;      // Reserved [- -]
} uBlox6_CFGRST_Payload;

// RXM configuration
#define UBLOX6_MSG_ID_CFGRXM 0x11
typedef struct {
    uint8_t reserved1;      // Always set to 8 [- -]
    uint8_t lpMode;         // Low Power Mode [- -]
                            // 0: Max. performance mode
                            // 1: Power Save Mode (>= FW 6.00 only)
                            // 2-3: reserved
                            // 4: Eco mode
                            // 5-255: reserved
} ublox6_CFGRXM_Payload;

// Set Message Rate
#define UBLOX6_MSG_ID_CFGMSG 0x01
typedef struct {
    uint8_t msgClass;       // Message Class [- -]
    uint8_t msgID;          // Message Identifier [- -]
    uint8_t rate;           // Send rate on current Target [- -]
} uBlox6_CFGMSG_Payload;

// Get/Set Navigation Engine Settings
#define UBLOX6_MSG_ID_CFGNAV5 0x24
typedef struct {
    uint16_t mask;          // Parameters Bitmask. Only the maskedparameters will be applied. [- -]
    uint8_t dynModel;		// Dynamic Platform model: [- -]
                            // 0 = Portable
                            // 2 = Stationary
                            // 3 = Pedestrian
                            // 4 = Automotive
                            // 5 = Sea
                            // 6 = Airborne with <1g Acceleration
                            // 7 = Airborne with <2g Acceleration
                            // 8 = Airborne with <4g Acceleration
    uint8_t fixMode;        // Position Fixing Mode. - 1: 2D only - 2: 3D only - 3: Auto 2D/3D [- -]
    int32_t fixedAlt;       // Fixed altitude (mean sea level) for 2D fix mode. [0.01 m]
    uint32_t fixedAltVar;   // Fixed altitude variance for 2D mode. [0.0001 m^2]
    int8_t minElev;         // Minimum Elevation for a GNSS satellite to be used in NAV [- deg]
    uint8_t drLimit;        // Maximum time to perform dead reckoning (linear extrapolation) in case of GPS signal loss [- s]
    uint16_t pDop;          // Position DOP Mask to use [0.1 -]
    uint16_t tDop;          // Time DOP Mask to use [0.1 -]
    uint16_t pAcc;          // Position Accuracy Mask [- m]
    uint16_t tAcc;          // Time Accuracy Mask [- m]
    uint8_t staticHoldThresh;   // Static hold threshold [- cm/s]
    uint8_t dgpsTimeOut;    // DGPS timeout, firmware 7 and newer only [- s]
    uint8_t cnoThreshNumSVs;// Number of satellites required to have C/N0 above cnoThresh for a valid fix. [- -]
    uint8_t cnoThresh;      // C/N0 threshold for a valid fix. [- dBHz]
    uint16_t reserved2;     // Always set to zero [- -]
    uint32_t reserved3;     // Always set to zero [- -]
    uint32_t reserved4;     // Always set to zero [- -]
} uBlox6_CFGNAV5_Payload;

// Navigation/Measurement Rate Settings
#define UBLOX6_MSG_ID_CFGRATE 0x08
typedef struct {
    uint16_t measRate;      // Measurement Rate, GPS measurements are taken every measRate milliseconds [- ms]
    uint16_t navRate;       // Navigation Rate, in number of measurement cycles. On u-blox 5 and u-blox 6, this parameter cannot be changed, and is always equals 1. [- cycles]
    uint16_t timeRef;       // Alignment to reference time: 0 = UTC time, 1 = GPS time [- -]
} uBlox6_CFGRATE_Payload;

// UTC Time Solution
#define UBLOX6_MSG_ID_NAVTIMEUTC 0x21
typedef struct {
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    uint32_t tAcc;          // Time Accuracy Estimate [- ns]
    int32_t nano;           // Nanoseconds of second, range -1e9 .. 1e9 (UTC) [- ns]
    uint16_t year;          // Year, range 1999..2099 (UTC) [- y]
    uint8_t month;          // Month, range 1..12 (UTC) [- month]
    uint8_t day;            // Day of Month, range 1..31 (UTC) [- d]
    uint8_t hour;           // Hour of Day, range 0..23 (UTC) [- h]
    uint8_t min;            // Minute of Hour, range 0..59 (UTC) [- min]
    uint8_t sec;            // Seconds of Minute, range 0..59 (UTC) [- s]
    uint8_t valid;          // Validity Flags (see graphic below) [- -]
} uBlox6_NAVTIMEUTC_Payload;

// Navigation Solution Information
#define UBLOX6_MSG_ID_NAVSOL 0x06
typedef struct {
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    int32_t fTOW;           // Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000 [- ns]
    int16_t week;           // GPS week (GPS time) [- -]
    uint8_t gpsFix;         // GPSfix Type, range 0..5 0x00 = No Fix 0x01 = Dead Reckoning only 0x02 = 2D-Fix 0x03 = 3D-Fix 0x04 = GPS + dead reckoning combined 0x05 = Time only fix 0x06..0xff: reserved [- -]
    uint8_t flags;          // Fix Status Flags (see graphic below) [- -]
    int32_t ecefX;          // ECEF X coordinate [- cm]
    int32_t ecefY;          // ECEF Y coordinate [- cm]
    int32_t ecefZ;          // ECEF Z coordinate [- cm]
    uint32_t pAcc;          // 3D Position Accuracy Estimate [- cm]
    int32_t ecefVX;         // ECEF X velocity [- cm/s]
    int32_t ecefVY;         // ECEF Y velocity [- cm/s]
    int32_t ecefVZ;         // ECEF Z velocity [- cm/s]
    uint32_t sAcc;          // Speed Accuracy Estimate [- cm/s]
    uint16_t pDOP;          // Position DOP [0.01 -]
    uint8_t reserved1;      // Reserved [- -]
    uint8_t numSV;          // Number of SVs used in Nav Solution [- -]
    uint32_t reserved2;     // Reserved [- -]
} uBlox6_NAVSOL_Payload;

// Geodetic Position Solution
#define UBLOX6_MSG_ID_NAVPOSLLH 0x02
typedef struct {
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    int32_t lon;            // Longitude [1e-7 deg]
    int32_t lat;            // Latitude [1e-7 deg]
    int32_t height;         // Height above Ellipsoid [- mm]
    int32_t hMSL;           // Height above mean sea level [- mm]
    uint32_t hAcc;          // Horizontal Accuracy Estimate [- mm]
    uint32_t vAcc;          // Vertical Accuracy Estimate [- mm]
} uBlox6_NAVPOSLLH_Payload;

// Velocity Solution in NED
#define UBLOX6_MSG_ID_NAVVELNED 0x12
typedef struct {
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    int32_t velN;           // NED north velocity [- cm/s]
    int32_t velE;           // NED east velocity [- cm/s]
    int32_t velD;           // NED down velocity [- cm/s]
    uint32_t speed;         // Speed (3-D) [- cm/s]
    uint32_t gSpeed;        // Ground Speed (2-D) [- cm/s]
    int32_t heading;        // Heading of motion 2-D [1e-5 deg]
    uint32_t sAcc;          // Speed Accuracy Estimate [- cm/s]
    uint32_t cAcc;          // Course / Heading Accuracy Estimate [1e-5 deg]
} uBlox6_NAVVELNED_Payload;

// Navigation Position Velocity Time Solution
// NAVPVT is unsupported on Ublox-G6010
#define UBLOX6_MSG_ID_NAVPVT 0x07
typedef struct {
    uint32_t iTOW;          // GPS time of week of the navigation epoch.See the description of iTOW for details. [- ms]
    uint16_t year;          // Year (UTC) [- y]
    uint8_t month;          // Month, range 1..12 (UTC) [- month]
    uint8_t day;            // Day of month, range 1..31 (UTC) [- day]
    uint8_t hour;           // Hour of day, range 0..23 (UTC) [- hour]
    uint8_t min;            // Minute of hour, range 0..59 (UTC) [- min]
    uint8_t sec;            // Seconds of minute, range 0..60 (UTC) [- sec]
    uint8_t valid;          // Validity Flags [- -]
    uint32_t tAcc;          // Time accuracy estimate (UTC) [- ns]
    int32_t nano;           // Fraction of second, range -1e9 .. 1e9 (UTC) [- ns]
    uint8_t fixType;        // GNSSfix Type, range 0..5 [- -]
                            // 0x00 = No Fix
                            // 0x01 = Dead Reckoning only
                            // 0x02 = 2D-Fix
                            // 0x03 = 3D-Fix
                            // 0x04 = GNSS + dead reckoning combined
                            // 0x05 = Time only fix
                            // 0x06..0xff: reserved
    uint8_t flags;          // Fix Status Flags [- -]
    uint8_t reserved1;      // Reserved [- -]
    uint8_t numSV;          // Number of satellites used in Nav Solution [- -]
    int32_t lon;            // Longitude [1e-7 deg]
    int32_t lat;            // Latitude [1e-7 deg]
    int32_t height;         // Height above Ellipsoid [- mm]
    int32_t hMSL;           // Height above mean sea level [- mm]
    uint32_t hAcc;          // Horizontal Accuracy Estimate [- mm]
    uint32_t vAcc;          // Vertical Accuracy Estimate [- mm]
    int32_t velN;           // NED north velocity [- mm/s]
    int32_t velE;           // NED east velocity [- mm/s]
    int32_t velD;           // NED down velocity [- mm/s]
    int32_t gSpeed;         // Ground Speed (2-D) [- mm/s]
    int32_t heading;        // Heading of motion 2-D [1e-5 deg]
    uint32_t sAcc;          // Speed Accuracy Estimate [- mm/s]
    uint32_t headingAcc;    // Heading Accuracy Estimate [1e-5 deg]
    uint16_t pDOP;          // Position DOP  [0.01 -]
    uint16_t reserved2;     // Reserved [- -]
    uint32_t reserved3;     // Reserved [- -]
} uBlox6_NAVPVT_Payload;

// Receiver Navigation Status
#define UBLOX6_MSG_ID_NAVSTATUS 0x03
typedef struct {
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    uint8_t gpsFix;         // GPS Fix Type, this value does not qualify a fix asvalid and within the limits. See note on flag gpsFixOk below. [- -]
    uint8_t flags;          // Navigation Status Flags, LSB = gpsFixOK - the only valid way of determining if a fix is actually OK. [- -]
    uint8_t fixStat;        // Fix Status Information [- -]
    uint8_t flags2;         // Power Save Mode State [- -]
    uint32_t ttff;          // Time to first fix (millisecond time tag) [- -]
    uint32_t msss;          // Milliseconds since Startup / Reset [- -]
} uBlox6_NAVSTATUS_Payload;

// UBX Checksum
typedef struct {
    uint8_t ck_a;           // Checksum A
    uint8_t ck_b;           // Checksum B
} uBlox6_Checksum;

// UBX Ack/Nack
typedef struct {
    uint8_t clsID;          // Message Class [- -]
    uint8_t msgID;          // Message Identifier [- -]
    uint8_t ck_a;           // Checksum A
    uint8_t ck_b;           // Checksum B
} uBlox6_ACKACK_Payload;

// Ublox UBX header
typedef struct  __attribute__((packed)){
    uint8_t sc1;            // 0xB5
    uint8_t sc2;            // 0x62
    uint8_t messageClass;   // Message Class [- -]
    uint8_t messageId;      // Message Identifier [- -]
    uint16_t payloadSize;   // Size of payload content
} uBlox6_Header;

// Handle all payloads
typedef union {
    uBlox6_CFGPRT_Payload cfgprt;
    uBlox6_CFGRST_Payload cfgrst;
    ublox6_CFGRXM_Payload cfgrxm;
    uBlox6_CFGMSG_Payload cfgmsg;
    uBlox6_CFGNAV5_Payload cfgnav5;
    uBlox6_NAVTIMEUTC_Payload navtimeutc;
    uBlox6_NAVSOL_Payload navsol;
    uBlox6_NAVPOSLLH_Payload navposllh;
    uBlox6_NAVVELNED_Payload navvelned;
    uBlox6_NAVSTATUS_Payload navstatus;
    uBlox6_ACKACK_Payload ackack;
} ublox6_PacketData;

// General packet for all payloads
typedef struct __attribute__((packed)){
    uBlox6_Header header;
    ublox6_PacketData data;
} uBlox6_Packet;

// General packet for all payloads without header
typedef struct __attribute__((packed)){
    ublox6_PacketData data;
} uBlox6_PacketPayload;

// User GPS data
typedef struct {
    uint16_t year;          // Year, range 1999..2099 (UTC) [- y]
    uint8_t month;          // Month, range 1..12 (UTC) [- month]
    uint8_t day;            // Day of Month, range 1..31 (UTC) [- d]
    uint8_t hour;           // Hour of Day, range 0..23 (UTC) [- h]
    uint8_t min;            // Minute of Hour, range 0..59 (UTC) [- min]
    uint8_t sec;            // Seconds of Minute, range 0..59 (UTC) [- s]
    uint8_t gpsFix;         // GPSfix Type
    uint8_t numSV;          // Number of SVs used in Nav Solution
    int32_t lon;            // Longitude [1e-7 deg]
    int32_t lat;            // Latitude [1e-7 deg]
    int32_t hMSL;           // Height above mean sea level [- mm]
    uint32_t speed;         // Speed (3-D) [- cm/s]
    uint32_t gSpeed;        // Ground Speed (2-D) [- cm/s]
    int32_t heading;        // Heading of motion 2-D [1e-5 deg]
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    int16_t week;           // GPS week (GPS time) [- -]
    uint32_t pAcc;          // 3D Position Accuracy Estimate [- cm]
    uint16_t pDOP;          // Position DOP [0.01 -]
    uint32_t sAcc;          // Speed Accuracy Estimate [- cm/s]
    uint8_t navSolFlags;    // NAV-SOL flags [- -]
    int32_t velD;           // Down velocity component in NED [- cm/s]
    uint8_t fullPacketFlag; // Information flag about full message received
} uBlox6_GPSData;

/* Decoder state */
typedef enum {
    UBX_DECODE_SYNC1 = 0,   // Sync1 found
    UBX_DECODE_SYNC2,       // Sync2 found
    UBX_DECODE_CLASS,       // Message class state
    UBX_DECODE_ID,          // Message ID state
    UBX_DECODE_LENGTH1,     // First length byte
    UBX_DECODE_LENGTH2,     // Second length byte
    UBX_DECODE_PAYLOAD,     // Payload content
    UBX_DECODE_CHKSUM1,     // First checksum byte
    UBX_DECODE_CHKSUM2,     // Second checksum byte
    UBX_DECODE_RTCM3        // RTCM3 decoding state, unused here
} uBlox6_decode_state;

/* Initialization state */
typedef enum {
    UBLOX6_INIT_RESET = 0,
    UBLOX6_INIT_PROTOCOL,
    UBLOX6_INIT_ALL
} uBlox6_init_state;

/* GPS fix states */
typedef enum {
    UBLOX6_GPSFIX_NONE = 0,
    UBLOX6_GPSFIX_DEAD_RECKONING = 1,
    UBLOX6_GPSFIX_2D_FIX = 2,
    UBLOX6_GPSFIX_3D_FIX = 3,
    UBLOX6_GPSFIX_GPS_DEAD_RECKONING = 4,
    UBLOX6_GPSFIX_TIME_ONLY_FIX = 5
} ublox6_GPSFixState;

// Functions
// Public
uint8_t Ublox6_Init(uBlox6_init_state init_state);
int Ublox6_HandleByte(uint8_t data);
uint8_t Ublox6_GetReceivedMsgCounter(void);
void Ublox6_ClearReceivedMsgCounter(void);
void Ublox6_GetLastData(uBlox6_GPSData *gpsEntry);
void Ublox6_Poll(uint8_t msgClass, uint8_t msgID);

#endif // UBLOX6_H
