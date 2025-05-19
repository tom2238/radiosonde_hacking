#include "amframe.h"

/**
 * @brief AMFrame::AMFrame
 * @param max_msg_size
 * @param max_chunk_size
 * @param max_rs_symbols
 * @param result_stat
 */
AMFrame::AMFrame(uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols, uint8_t *result_stat) {
    // Init SSF Reed Solomon
    fec_rs_object = new SSFRS(max_msg_size,max_chunk_size,max_rs_symbols,result_stat);
}

AMFrame::AMFrame(void) {
    if(fec_rs_object) {
        delete fec_rs_object;
    }
}

/**
 * @brief AMFrame::~AMFrame
 */
AMFrame::~AMFrame(void) {
    if(fec_rs_object) {
        delete fec_rs_object;
    }
}

int AMFrame::InitSSF(uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols) {
    // Init SSF Reed Solomon
    uint8_t rslt;
    fec_rs_object = new SSFRS(max_msg_size,max_chunk_size,max_rs_symbols,&rslt);
    return rslt;
}

// NRZ
//{ 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8} transmitted in frame
//{ 0x86, 0x35, 0xF4, 0x40, 0x93, 0xDF, 0x1A, 0x60} XORed in receiver
// Manchester
//{ 0x9A, 0x99, 0x99, 0x99, 0xA9, 0x6D, 0x55, 0x55} transmitted in frame
// No XORing, scrambling
/**
 * @brief AMFrame::NewFrameData
 * @param frame_length
 * @param modulation
 * @return
 */
FrameData AMFrame::NewFrameData(int frame_length, FrameModulation modulation) {
    FrameData newframe;
    newframe.modulation = modulation;
    if(newframe.modulation == FRAME_MOD_NRZ) {
        newframe.value[0] = 0x86;
        newframe.value[1] = 0x35;
        newframe.value[2] = 0xF4;
        newframe.value[3] = 0x40;
        newframe.value[4] = 0x93;
        newframe.value[5] = 0xDF;
        newframe.value[6] = 0x1A;
        newframe.value[7] = 0x60;
    } else if (newframe.modulation == FRAME_MOD_MAN) {
        newframe.value[0] = 0x9A;
        newframe.value[1] = 0x99;
        newframe.value[2] = 0x99;
        newframe.value[3] = 0x99;
        newframe.value[4] = 0xA9;
        newframe.value[5] = 0x6D;
        newframe.value[6] = 0x55;
        newframe.value[7] = 0x55;
    } else {
        newframe.value[0] = 0x00;
        newframe.value[1] = 0x00;
        newframe.value[2] = 0x00;
        newframe.value[3] = 0x00;
        newframe.value[4] = 0x00;
        newframe.value[5] = 0x00;
        newframe.value[6] = 0x00;
        newframe.value[7] = 0x00;
    }
    newframe.length = frame_length;

    int i;
    for(i=8;i<newframe.length;i++) {
        newframe.value[i] = 0;
    }
    return newframe;
}

/**
 * @brief AMFrame::NewFrameHead
 * @param modulation
 * @return
 */
FrameHead AMFrame::NewFrameHead(FrameModulation modulation) {
    FrameHead newhead;
    newhead.modulation = modulation;
    // little endian
    // NRZ header
    newhead.header =    "0000100001101101010100111000100001000100011010010100100000011111";
    // little endian
    // Manchester header  01->1,10->0
    newhead.header_mc = "0101100110011001100110011001100110010101101101101010101010101010";
    newhead.position = -1;

    int i;
    for(i=0;i<HEAD_LEN+1;i++) {
        newhead.value[i] = 'x';
    }
    return newhead;
}

/**
 * @brief AMFrame::IncHeadPos
 * @param incpos
 */
void AMFrame::IncHeadPos(FrameHead *incpos) {
    incpos->position = (incpos->position+1) % HEAD_LEN;
}

/**
 * @brief AMFrame::FrameHeadCompare
 * @param head
 * @return
 */
int AMFrame::FrameHeadCompare(FrameHead head) {
    int i = 0;
    int j = head.position;

    while (i < HEAD_LEN) {
        if (j < 0) {
            j = HEAD_LEN-1;
        }
        if(head.modulation == FRAME_MOD_NRZ) {
            if (head.value[j] != head.header[HEAD_OFS+HEAD_LEN-1-i]) {
                break;
            }
        } else {
            if (head.value[j] != head.header_mc[HEAD_OFS+HEAD_LEN-1-i]) {
                break;
            }
        }
        j--;
        i++;
    }

    return i;
}

/**
 * @brief AMFrame::PrintFrameData
 * @param frame
 * @param ecc_size_bytes
 */
void AMFrame::PrintFrameData(FrameData frame, int ecc_size_bytes) {
    int i;
    int lines = 0;
    uint16_t crctrs = GetCRC16(frame,ecc_size_bytes);
    uint16_t crcrec = CalculateCRC16(&frame,ecc_size_bytes); // Calculate rewrite internal CRC value
    printf("%2d: ",lines);
    for(i=0;i<frame.length;i++) {
        printf("%02X ",frame.value[i]);
        if((i+1)%16==0) {
            printf("\n");
            lines++;
            printf("%2d: ",lines);
        }
    }
    if(crcrec==crctrs) {
        printf(" CRC OK ");
    } else {
        printf(" CRC FAIL ");
    }
    printf("CRC(rt) %x : %x\n",crcrec,crctrs);
}

/**
 * @brief PrintFrame_STM32
 * @param frame
 * @param ecc_size_bytes
 */
void AMFrame::PrintFrame_STM32(FrameData frame, int ecc_size_bytes) {
    // Calculate CRC
    uint16_t crctrs = GetCRC16(frame,ecc_size_bytes);
    uint16_t crcrec = CalculateCRC16(&frame,ecc_size_bytes); // Calculate rewrite internal CRC value
    // Get values from STM32
    // Current frame number
    unsigned int stm_frame_count = 0;
    stm_frame_count = (frame.value[8]) << 24;
    stm_frame_count += (frame.value[9]) << 16;
    stm_frame_count += (frame.value[10]) << 8;
    stm_frame_count += (frame.value[11]) << 0;
    // ADC Vref voltage in milivolts
    uint16_t stm_adc_vref = 0;
    stm_adc_vref = (frame.value[12]) << 8;
    stm_adc_vref += (frame.value[13]) << 0;
    float stm_adc_vref_f = (float)(stm_adc_vref) / 1000; // In volts
    // ADC voltage on channel 0, pin PA0 in milivolts
    uint16_t stm_adc_ch0 = 0;
    stm_adc_ch0 = (frame.value[14]) << 8;
    stm_adc_ch0 += (frame.value[15]) << 0;
    float stm_adc_ch0_f = (float)(stm_adc_ch0) / 1000; // In volts
    // ADC MCU temperature in centi celsius degree
    uint16_t stm_adc_temp = 0;
    stm_adc_temp = (frame.value[16]) << 8;
    stm_adc_temp += (frame.value[17]) << 0;
    float stm_adc_temp_f = (float)(stm_adc_temp) / 100; // In deg C
    // ADC MCU supply voltage in milivolts
    uint16_t stm_adc_supply = 0;
    stm_adc_supply = (frame.value[18]) << 8;
    stm_adc_supply += (frame.value[19]) << 0;
    float stm_adc_supply_f = (float)(stm_adc_supply) / 1000; // In volts
    fprintf(stdout,"[%d], Power supply: %.3f V, Reference: %.3f V, ADC channel 0: %.3f V, MCU temperature: %.1f °C, ",stm_frame_count,stm_adc_supply_f,stm_adc_vref_f,stm_adc_ch0_f,stm_adc_temp_f);
    // Check CRC value
    if(crcrec==crctrs) {
        printf("[CRC OK]\n");
    } else {
        printf("[CRC FAIL]\n");
    }
}

/**
 * @brief AMFrame::PrintFrame_RS41GPS
 * @param frame
 * @param ecc_size_bytes
 */
void AMFrame::PrintFrame_RS41GPS(FrameData frame, int ecc_size_bytes) {
    // Calculate CRC
    uint16_t crctrs = GetCRC16(frame,ecc_size_bytes);
    uint16_t crcrec = CalculateCRC16(&frame,ecc_size_bytes); // Calculate rewrite internal CRC value
    // RS41 ublox GPS data
    uint16_t year;          // Year, range 1999..2099 (UTC) [- y]
    year = (frame.value[8]) << 24;
    year += (frame.value[9]) << 16;
    year += (frame.value[10]) << 8;
    year += (frame.value[11]) << 0;
    uint8_t month;          // Month, range 1..12 (UTC) [- month]
    month = frame.value[12];
    uint8_t day;            // Day of Month, range 1..31 (UTC) [- d]
    day = frame.value[13];
    uint8_t hour;           // Hour of Day, range 0..23 (UTC) [- h]
    hour = frame.value[14];
    uint8_t min;            // Minute of Hour, range 0..59 (UTC) [- min]
    min = frame.value[15];
    uint8_t sec;            // Seconds of Minute, range 0..59 (UTC) [- s]
    sec = frame.value[16];
    uint8_t gpsFix;         // GPSfix Type
    gpsFix = frame.value[17];
    uint8_t numSV;          // Number of SVs used in Nav Solution
    numSV = frame.value[18];
    int32_t lon;            // Longitude [1e-7 deg]
    lon = (frame.value[19]) << 24;
    lon += (frame.value[20]) << 16;
    lon += (frame.value[21]) << 8;
    lon += (frame.value[22]) << 0;
    double lon_f = ((float)(lon))*1e-7;
    int32_t lat;            // Latitude [1e-7 deg]
    lat = (frame.value[23]) << 24;
    lat += (frame.value[24]) << 16;
    lat += (frame.value[25]) << 8;
    lat += (frame.value[26]) << 0;
    double lat_f = ((float)(lat))*1e-7;
    int32_t hMSL;           // Height above mean sea level [- mm]
    hMSL = (frame.value[27]) << 24;
    hMSL += (frame.value[28]) << 16;
    hMSL += (frame.value[29]) << 8;
    hMSL += (frame.value[30]) << 0;
    float alt_f = ((float)(hMSL))*1e-3;
    uint32_t speed;         // Speed (3-D) [- cm/s]
    speed = (frame.value[31]) << 24;
    speed += (frame.value[32]) << 16;
    speed += (frame.value[33]) << 8;
    speed += (frame.value[34]) << 0;
    float speed_f = ((float)(speed))/100;
    uint32_t gSpeed;        // Ground Speed (2-D) [- cm/s]
    gSpeed = (frame.value[35]) << 24;
    gSpeed += (frame.value[36]) << 16;
    gSpeed += (frame.value[37]) << 8;
    gSpeed += (frame.value[38]) << 0;
    float gSpeed_f = ((float)(gSpeed))/100;
    int32_t heading;        // Heading of motion 2-D [1e-5 deg]
    heading = (frame.value[39]) << 24;
    heading += (frame.value[40]) << 16;
    heading += (frame.value[41]) << 8;
    heading += (frame.value[42]) << 0;
    float heading_f = ((float)(heading))*1e-5;
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    iTOW = (frame.value[43]) << 24;
    iTOW += (frame.value[44]) << 16;
    iTOW += (frame.value[45]) << 8;
    iTOW += (frame.value[46]) << 0;
    // Calculate day
    uint32_t gpstime = iTOW/1000;
    uint32_t gpsday = (gpstime / (24 * 3600)) % 7;
    const char weekday[7][3] = { "Su", "Mo", "Th", "We", "Tr", "Fr", "Sa"};
    int16_t week;           // GPS week (GPS time) [- -]
    week = (frame.value[47]) << 8;
    week += (frame.value[48]) << 0;
    uint16_t pDOP;          // Position DOP [0.01 -]
    pDOP = (frame.value[49]) << 24;
    pDOP += (frame.value[50]) << 16;
    pDOP += (frame.value[51]) << 8;
    pDOP += (frame.value[52]) << 0;
    // Frame count
    uint16_t frame_cnt;
    frame_cnt = (frame.value[53]) << 8;
    frame_cnt += (frame.value[54]);
    // Sonde ID
    char SondeID[8];
    SondeID[0] = frame.value[55];
    SondeID[1] = frame.value[56];
    SondeID[2] = frame.value[57];
    SondeID[3] = frame.value[58];
    SondeID[4] = frame.value[59];
    SondeID[5] = frame.value[60];
    SondeID[6] = frame.value[61];
    SondeID[7] = frame.value[62];

    // Print
    //fprintf(stdout,"[GPS] %02d.%02d.%04d %02d:%02d:%02d Fix:%d, numSV:%d, Lat:%.7f, Lon:%.7f Alt:%.2f Speed:%.1f Ground speed:%.1f Heading:%.1f ",day,month,year,hour,min,sec,gpsFix,numSV,lat_f,lon_f,alt_f,speed_f,gSpeed_f,heading_f);

    // Check CRC value
    if(crcrec==crctrs) {
        //printf("[CRC OK]\n");
        //[ 5653] (R5030250) So 2021-09-05 12:33:28.001 (W 2174)  lat: 48.82555  lon: 20.93063  alt: 25231.48   vH: 13.8  D:  71.1  vV: 7.0 //
        fprintf(stdout,"[%d] (%s) %s %04d-%02d-%02d %02d:%02d:%02d.001 (W %d)  lat: %.7f  lon: %.7f  alt: %.2f  vH: %.1f  D: %.1f  vV: %.1f  numSV: %d\n",frame_cnt,SondeID,weekday[gpsday],year,month,day,hour,min,sec,week,lat_f,lon_f,alt_f,gSpeed_f,heading_f,0.0f,numSV);
    } else {
        //printf("[CRC FAIL]\n");
    }
    fflush(stdout);
}

/**
 * @brief AMFrame::PrintFrame_RS41Sounding
 * @param frame
 * @param ecc_size_bytes
 */
void AMFrame::PrintFrame_RS41Sounding(FrameData frame, int ecc_size_bytes) {
    // Calculate CRC
    uint16_t crctrs = GetCRC16(frame,ecc_size_bytes);
    uint16_t crcrec = CalculateCRC16(&frame,ecc_size_bytes); // Calculate rewrite internal CRC value

    // RS41 ublox GPS data
    uint16_t year;          // Year, range 1999..2099 (UTC) [- y]
    year = (frame.value[8]) << 24;
    year += (frame.value[9]) << 16;
    year += (frame.value[10]) << 8;
    year += (frame.value[11]) << 0;
    uint8_t month;          // Month, range 1..12 (UTC) [- month]
    month = frame.value[12];
    uint8_t day;            // Day of Month, range 1..31 (UTC) [- d]
    day = frame.value[13];
    uint8_t hour;           // Hour of Day, range 0..23 (UTC) [- h]
    hour = frame.value[14];
    uint8_t min;            // Minute of Hour, range 0..59 (UTC) [- min]
    min = frame.value[15];
    uint8_t sec;            // Seconds of Minute, range 0..59 (UTC) [- s]
    sec = frame.value[16];
    uint8_t gpsFix;         // GPSfix Type
    gpsFix = frame.value[17];
    uint8_t numSV;          // Number of SVs used in Nav Solution
    numSV = frame.value[18];
    int32_t lon;            // Longitude [1e-7 deg]
    lon = (frame.value[19]) << 24;
    lon += (frame.value[20]) << 16;
    lon += (frame.value[21]) << 8;
    lon += (frame.value[22]) << 0;
    double lon_f = ((float)(lon))*1e-7;
    int32_t lat;            // Latitude [1e-7 deg]
    lat = (frame.value[23]) << 24;
    lat += (frame.value[24]) << 16;
    lat += (frame.value[25]) << 8;
    lat += (frame.value[26]) << 0;
    double lat_f = ((float)(lat))*1e-7;
    int32_t hMSL;           // Height above mean sea level [- mm]
    hMSL = (frame.value[27]) << 24;
    hMSL += (frame.value[28]) << 16;
    hMSL += (frame.value[29]) << 8;
    hMSL += (frame.value[30]) << 0;
    float alt_f = ((float)(hMSL))*1e-3;
    int32_t vspeed;         // Down velocity component [- cm/s]
    vspeed = (frame.value[31]) << 24;
    vspeed += (frame.value[32]) << 16;
    vspeed += (frame.value[33]) << 8;
    vspeed += (frame.value[34]) << 0;
    float vspeed_f = ((float)(vspeed))/100;
    uint32_t gSpeed;        // Ground Speed (2-D) [- cm/s]
    gSpeed = (frame.value[35]) << 24;
    gSpeed += (frame.value[36]) << 16;
    gSpeed += (frame.value[37]) << 8;
    gSpeed += (frame.value[38]) << 0;
    float gSpeed_f = ((float)(gSpeed))/100;
    int32_t heading;        // Heading of motion 2-D [1e-5 deg]
    heading = (frame.value[39]) << 24;
    heading += (frame.value[40]) << 16;
    heading += (frame.value[41]) << 8;
    heading += (frame.value[42]) << 0;
    float heading_f = ((float)(heading))*1e-5;
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    iTOW = (frame.value[43]) << 24;
    iTOW += (frame.value[44]) << 16;
    iTOW += (frame.value[45]) << 8;
    iTOW += (frame.value[46]) << 0;
    // Calculate day
    uint32_t gpstime = iTOW/1000;
    uint32_t gpsday = (gpstime / (24 * 3600)) % 7;
    const char weekday[7][3] = { "Su", "Mo", "Th", "We", "Tr", "Fr", "Sa"};
    int16_t week;           // GPS week (GPS time) [- -]
    week = (frame.value[47]) << 8;
    week += (frame.value[48]) << 0;
    // PTU main temperature only
    uint32_t ptu_main_sensor;
    ptu_main_sensor = (frame.value[49]) << 24;
    ptu_main_sensor += (frame.value[50]) << 16;
    ptu_main_sensor += (frame.value[51]) << 8;
    ptu_main_sensor += (frame.value[52]) << 0;
    // Calculate temperaure
    float ptu_main_sensor_f = (float)(ptu_main_sensor);
    ptu_main_sensor_f = (ptu_main_sensor_f / 100) - 100;
    // Get voltage
    uint8_t bat_voltage = frame.value[53];
    float bat_voltage_f = ((float)(bat_voltage))/10;
    // Sonde ID
    char SondeID[8];
    SondeID[0] = frame.value[54];
    SondeID[1] = frame.value[55];
    SondeID[2] = frame.value[56];
    SondeID[3] = frame.value[57];
    SondeID[4] = frame.value[58];
    SondeID[5] = frame.value[59];
    SondeID[6] = frame.value[60];
    SondeID[7] = frame.value[61];
    // Frame count
    uint16_t frame_cnt;
    frame_cnt = (frame.value[62]) << 8;
    frame_cnt += (frame.value[63]);
    // Frequency MHz
    uint16_t freq_mhz = (frame.value[64]) << 8;
    freq_mhz += frame.value[65];
    float freq_mhz_f = ((float)(freq_mhz))/100;
    // Txpower dBm
    uint8_t tx_power = frame.value[66];

    // Check CRC value
    if(crcrec==crctrs) {
        if(numSV == 0) {
            lat_f = 0.0f;
            lon_f = 0.0f;
        }

        //printf("[CRC OK]\n");
        //[ 5653] (R5030250) So 2021-09-05 12:33:28.001 (W 2174)  lat: 48.82555  lon: 20.93063  alt: 25231.48   vH: 13.8  D:  71.1  vV: 7.0 //
        //fprintf(stdout,"[%d] (%s) %s %04d-%02d-%02d %02d:%02d:%02d.001 (W %d)  lat: %.7f  lon: %.7f  alt: %.2f  vH: %.1f  D: %.1f  vV: %.1f  numSV: %d  Tm: %.1f  vBat: %.1f  freq: %.3f  txPower: %d\n",frame_cnt,SondeID,weekday[gpsday],year,month,day,hour,min,sec,week,lat_f,lon_f,alt_f,gSpeed_f,heading_f,vspeed_f,numSV,ptu_main_sensor_f,bat_voltage_f,freq_mhz_f,tx_power);

        // Print UKHAS string
        char ukhas_msg[512];
        uint16_t ukhas_crc = 0;
        // With comment
        //int chars_writed = snprintf(ukhas_msg,sizeof(ukhas_msg),"$$%s,%d,%02d:%02d:%02d,%.7f,%.7f,%.0f,%.1f,%.1f,%.1f,%.0f,%.1f,%d,%.3f MHz,https://github.com/tom2238/radiosonde_hacking/tree/main/rs41/sounding_sonde",SondeID,frame_cnt,hour,min,sec,lat_f,lon_f,alt_f,gSpeed_f,ptu_main_sensor_f,bat_voltage_f,heading_f,vspeed_f,numSV,freq_mhz_f);
        // Without comment and CRC
        int chars_writed = snprintf(ukhas_msg,sizeof(ukhas_msg),"$$%s,%d,%02d:%02d:%02d,%.7f,%.7f,%.0f,%.1f,%.1f,%.1f,%.0f,%.1f,%d,%.3f MHz",SondeID,frame_cnt,hour,min,sec,lat_f,lon_f,alt_f,gSpeed_f,ptu_main_sensor_f,bat_voltage_f,heading_f,vspeed_f,numSV,freq_mhz_f);
        if(chars_writed < sizeof(ukhas_msg)) {
            // CRC is calculated in habitat_upload
            //ukhas_crc = ukhas_CRC16_checksum(ukhas_msg);
        } else {
            fprintf(stderr,"snprinf buffer length error\n");
        }
        //fprintf(stdout,"%s*%04x\n",ukhas_msg,ukhas_crc);
        fprintf(stdout,"%s\n",ukhas_msg);
    } else {
        //printf("[CRC FAIL]\n");
    }
    fflush(stdout);
}

/**
 * @brief AMFrame::XOR
 * @param frame
 * @param start
 */
void AMFrame::XOR(FrameData *frame, int start) {
    const uint8_t mask[FRAME_XORMASK_LEN] = { 0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
                                              0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
                                              0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
                                              0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
                                              0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
                                              0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
                                              0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
                                              0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1};
    /* LFSR: ab i=8 (mod 64):
   * m[16+i] = m[i] ^ m[i+2] ^ m[i+4] ^ m[i+6]
   * ________________3205590EF944C6262160C2EA795D6DA15469470CDCE85CF1
   * F776827F0799A22C937C3063F5102E61D0BCB4B606AAF423786E3BAEBF7B4CC196833E51B1490898
   */
    int i;
    if(start > frame->length || start < 0) {
        start = 0;
    }
    for(i=start;i<frame->length;i++) {
        frame->value[i] = frame->value[i] ^ mask[i % FRAME_XORMASK_LEN];
    }
}

/**
 * @brief AMFrame::WriteToFile
 * @param frame
 * @param fp
 * @param ecc_size_bytes
 */
void AMFrame::WriteToFile(FrameData frame, FILE *fp, int ecc_size_bytes) {
    int i;
    for(i=FRAME_START+1;i<frame.length-CRC_SIZE-ecc_size_bytes;i++) {
        fwrite(&frame.value[i], 1, 1, fp);
    }
}

/**
 * @brief AMFrame::CalculateCRC16
 * @param frame
 * @param ecc_size_bytes
 * @return
 */
uint16_t AMFrame::CalculateCRC16(FrameData *frame, int ecc_size_bytes) {
    // CRC-16/CCITT-FALSE
    int crc = 0xFFFF;          // initial value
    int polynomial = 0x1021;   // 0001 0000 0010 0001  (0, 5, 12)
    int i,j;
    uint8_t byte;
    for (i=frame->length-CRC_SIZE-ecc_size_bytes;i>FRAME_START+1;i--) {
        byte = frame->value[i-1] & 0xFF;
        for (j=0;j<8;j++) {
            uint8_t bit = ((byte >> (7-j) & 1) == 1);
            uint8_t c15 = ((crc >> 15 & 1) == 1);
            crc <<= 1;
            if (c15 ^ bit) {
                crc ^= polynomial;
            }
        }
    }
    crc &= 0xFFFF;
    frame->value[frame->length-1-ecc_size_bytes] = crc & 0xFF;
    frame->value[frame->length-2-ecc_size_bytes] = (crc >> 8) & 0xFF;
    return crc;
}

/**
 * @brief AMFrame::crc_xmodem_update
 * @param crc
 * @param data
 * @return
 */
uint16_t AMFrame::crc_xmodem_update(uint16_t crc, uint8_t data) {
    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for (i=0; i<8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

/**
 * @brief AMFrame::ukhas_CRC16_checksum
 * @param string
 * @return
 */
uint16_t AMFrame::ukhas_CRC16_checksum (char *string) {
    size_t i;
    uint16_t crc;
    uint8_t c;

    crc = 0xFFFF;

    // Calculate checksum ignoring the first two $s
    for (i = 2; i < strlen(string); i++)
    {
        c = string[i];
        crc = crc_xmodem_update (crc, c);
    }

    return crc;
}

/**
 * @brief AMFrame::GetCRC16
 * @param frame
 * @param ecc_size_bytes
 * @return
 */
uint16_t AMFrame::GetCRC16(FrameData frame, int ecc_size_bytes) {
    uint16_t lsb = frame.value[frame.length-1-ecc_size_bytes] & 0xFF;
    uint16_t msb = (frame.value[frame.length-2-ecc_size_bytes] << 8) & 0xFF00;
    return lsb + msb;
}

/**
 * @brief AMFrame::ManchesterEncode
 * @param frame
 * @param start
 * @return
 */
int AMFrame::ManchesterEncode(FrameData *frame, int start) {
    int i,j;
    int ManFramePosition = FRAME_START+1;
    int ManBitPosition = 0;
    char Manbitbuf[8];
    uint8_t ManByte;
    uint8_t byte;
    uint8_t frame_bits[8];
    FrameData ManEncode = NewFrameData(frame->length*2, FRAME_MOD_MAN);
    //printf("Manchester frame len: %d\n",ManEncode.length);
    for(i=start;i<frame->length;i++) {
        byte = frame->value[i];
        for(j=0;j<8;j++) {
            frame_bits[j] = (byte >> j) & 0x01;
            // Manchester  01->1,10->0
            if(frame_bits[j] == 1) { // Bit Is 1
                Manbitbuf[ManBitPosition] = 0;
                ManBitPosition++;
                Manbitbuf[ManBitPosition] = 1;
                ManBitPosition++;
            } else { // Bit Is 0
                Manbitbuf[ManBitPosition] = 1;
                ManBitPosition++;
                Manbitbuf[ManBitPosition] = 0;
                ManBitPosition++;
            }
            if(ManBitPosition >= 8) {
                ManByte = (uint8_t)Bits2Byte(Manbitbuf);
                ManBitPosition = 0;
                ManEncode.value[ManFramePosition] = ManByte;
                ManFramePosition++;
            }
        }
    }
    // Rewrite frame
    frame->length = (frame->length*2)-HEAD_SIZE;
    frame->modulation = FRAME_MOD_MAN;
    for(i=0;i<frame->length;i++){
        frame->value[i] = ManEncode.value[i];
    }

    return 0;
}

/**
 * @brief AMFrame::ManchesterDecode
 * @param frame
 * @param start
 * @return
 */
int AMFrame::ManchesterDecode(FrameData *frame, int start) {
    int i,j;
    int DataFramePosition = start;
    int DataBitPosition = 0;
    unsigned int errorCount = 0;
    char Databitbuf[8];
    uint8_t DataByte;
    uint8_t byte;
    uint8_t frame_bits[8];
    uint8_t symbolCounter = 0;
    uint8_t symbolCode[2];
    FrameData DataDecode = NewFrameData(frame->length, FRAME_MOD_MAN);
    for(i=start;i<frame->length;i++) {
        byte = frame->value[i];
        for(j=0;j<8;j++) {
            frame_bits[j] = (byte >> j) & 0x01;
            // Manchester  01->1,10->0
            symbolCode[symbolCounter] = frame_bits[j];
            symbolCounter++;
            if(symbolCounter >= 2) {
                if(symbolCode[0]==0 && symbolCode[1]==1) { // Decoded 1
                    Databitbuf[DataBitPosition] = 1;
                } else if (symbolCode[0]==1 && symbolCode[1]==0) { // Decoded 0
                    Databitbuf[DataBitPosition] = 0;
                } else { // Some error, one symbol shift or isnot manchester
                    Databitbuf[DataBitPosition] = 0;
                    errorCount++;
                }
                DataBitPosition++;
                symbolCounter = 0;
            }
            if(DataBitPosition >= 8) {
                DataByte = (uint8_t)Bits2Byte(Databitbuf);
                DataBitPosition = 0;
                DataDecode.value[DataFramePosition] = DataByte;
                DataFramePosition++;
            }
        }
    }
    // Rewrite frame
    frame->length = (frame->length+HEAD_SIZE)/2;
    frame->modulation = FRAME_MOD_NRZ;
    for(i=0;i<frame->length;i++){
        frame->value[i] = DataDecode.value[i];
    }
    return errorCount;
}

/**
 * @brief AMFrame::Bits2Byte
 * @param bits
 * @return
 */
int AMFrame::Bits2Byte(char bits[]) {
    int i, byteval=0, d=1;
    for (i = 0; i < 8; i++) {     // little endian
        /* for (i = 7; i >= 0; i--) { // big endian */
        if      (bits[i] == 1)  {
            byteval += d;
        }
        else if (bits[i] == 0)  {
            byteval += 0;
        }
        else {
            return 0x100;
        }
        d <<= 1;
    }
    return byteval;
}

/**
 * @brief AMFrame::CheckRSLimit
 * @param msg_len
 * @param parity_len
 * @return
 */
uint8_t AMFrame::CheckRSLimit(uint16_t msg_len, uint16_t parity_len) {
    if((msg_len + parity_len) > FRAME_SSF_RS_TOTAL_CHUNK_LIMIT) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief AMFrame::RSEncode
 * @param frame
 */
void AMFrame::RSEncode(FrameData *frame) {
    int i;
    uint16_t eccLen;
    uint16_t ecc_rs_size = fec_rs_object->GetRSSize();
    uint16_t chunk_size = fec_rs_object->GetChunkSize();
    // Data + CRC bytes array
    uint8_t msg[frame->length-ecc_rs_size-HEAD_SIZE];
    // Copy only Data + CRC
    int j = 0;
    for(i=FRAME_START+1;i<frame->length-ecc_rs_size;i++) {
        msg[j] = frame->value[i];
        j++;
    }
    // Array for RS parity bytes
    uint8_t ecc[ecc_rs_size];
    // Encode Reed-Solomon
    fec_rs_object->Encode(msg, (uint16_t)sizeof(msg), ecc, (uint16_t)sizeof(ecc), &eccLen, ecc_rs_size, chunk_size);
    // Copy parity bytes to frame
    j = 0;
    for(i=frame->length-ecc_rs_size;i<frame->length;i++) {
        frame->value[i] = ecc[j];
        j++;
    }
}

/**
 * @brief AMFrame::RSDecode
 * @param frame
 */
void AMFrame::RSDecode(FrameData *frame) {
    int i;
    uint16_t msgLen;
    uint16_t ecc_rs_size = fec_rs_object->GetRSSize();
    uint16_t chunk_size = fec_rs_object->GetChunkSize();
    // Data + CRC + parity array
    uint8_t msgRx[frame->length-HEAD_SIZE];
    // Copy only Data + CRC + parity
    int j = 0;
    for(i=FRAME_START+1;i<frame->length;i++) {
        msgRx[j] = frame->value[i];
        j++;
    }
    // Error correction on the received message
    fec_rs_object->Decode(msgRx, (uint16_t)sizeof(msgRx), &msgLen, ecc_rs_size, chunk_size);
    // Copy decoded Data + CRC to frame
    j = 0;
    for(i=FRAME_START+1;i<frame->length-ecc_rs_size;i++) {
        frame->value[i] = msgRx[j];
        j++;
    }
}
