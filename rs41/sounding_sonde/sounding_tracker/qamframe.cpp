#include "qamframe.h"

/**
 * @brief QAMFrame::QAMFrame
 * @param parent
 */
QAMFrame::QAMFrame(QObject *parent) : QObject(parent) {
    // New frame without SSF
    amframe = new AMFrame();
    ssf_enable = false;
    // Init decoded data
    frame_decoded.insert("GPS_YEAR","1970");
    frame_decoded.insert("GPS_MONTH","1");
    frame_decoded.insert("GPS_DAY","1");
    frame_decoded.insert("GPS_HOUR","0");
    frame_decoded.insert("GPS_MINUTE","0");
    frame_decoded.insert("GPS_SECOND","0");
    frame_decoded.insert("GPS_FIX","0");
    frame_decoded.insert("GPS_NUMSV","0");
    frame_decoded.insert("GPS_LON","0.000000");
    frame_decoded.insert("GPS_LAT","0.000000");
    frame_decoded.insert("GPS_ALT","0.0");
    frame_decoded.insert("GPS_CLIMBING","0.00");
    frame_decoded.insert("GPS_GROUNDSPEED","0.00");
    frame_decoded.insert("GPS_HEADING","0");
    frame_decoded.insert("TEMPERATURE_MAIN","0.0");
    frame_decoded.insert("SONDE_ID","UNKNOWN");
    frame_decoded.insert("FRAME_COUNT","0");
    frame_decoded.insert("ONBOARD_VOLTAGE","0.0");
    frame_decoded.insert("TX_FREQUENCY","433.125");
    frame_decoded.insert("TX_POWER","10");

    qDebug() << "QMap list" << frame_decoded.value("GPS_YEAR") << "s:" << frame_decoded.size();

}

/**
 * @brief QAMFrame::~QAMFrame
 */
QAMFrame::~QAMFrame(void) {
    delete amframe;
}

/**
 * @brief QAMFrame::Init
 * @param res
 * @param inv
 * @param avg
 */
bool QAMFrame::Init(bool res, bool inv, bool avg, QAudioFormat audio_format, int baudrate, int frame_length, FrameModulation modulation, int ecc_level) {
    // Init all
    // Check audio params
    if(audio_format.channelCount() != 1) {
        return true;
    }
    if(audio_format.sampleSize() != 16) {
        return true;
    }
    if(QString::compare(audio_format.codec(),"audio/pcm") != 0) {
        return true;
    }
    if(audio_format.sampleType() != QAudioFormat::SignedInt) {
        return true;
    }

    // Samples and bits
    s_baudrate = baudrate;
    s_samples_per_bit = static_cast<float>(audio_format.sampleRate()) / static_cast<float>(s_baudrate);

    // Frame length and type
    frm_length = frame_length + HEAD_SIZE + ECC_SIZE + CRC_SIZE;
    frm_modulation = modulation;

    // Set Reed-Solomon parity size
    switch (ecc_level) {
    case 6: // Level 6 = (255,207)
        ecc_code = 48;
        break;
    case 5: // Level 5 = (255,215)
        ecc_code = 40;
        break;
    case 4: // Level 4 = (255,223)
        ecc_code = 32;
        break;
    case 3: // Level 3 = (255,231)
        ecc_code = 24;
        break;
    case 2: // Level 2 = (255,239)
        ecc_code = 16;
        break;
    case 1: // Level 1 = (255,247)
        ecc_code = 8;
        break;
    default: // Level 0 = uncoded
        ecc_code = 0;
    }

    // Add ECC into frame length
    frm_length += ecc_code;

    // Check ECC max size
    if((amframe->CheckRSLimit(frm_length-HEAD_SIZE-ecc_code,ecc_code)) && (ecc_code!=0)) {
        fprintf(stderr,"Error: Frame length %d is too long for Reed-Solomon coding. Please limit in: DATA(%d) + CRC(%d) + PARITY(%d) <= 255 bytes.\n",frm_length,frm_length-HEAD_SIZE-CRC_SIZE-ecc_code,CRC_SIZE,ecc_code);
        return true;
    }
    // Handle Reed-Solomon en/decoder init
    if(ecc_code!=0) {
        if(InitSSF(frm_length-ecc_code-HEAD_SIZE,frm_length-ecc_code-HEAD_SIZE,ecc_code)) {
            fprintf(stderr,"Error: Initializing Reed-Solomon en/decoder.\n");
            return true;
        }
    }
    // Check frame length
    if(frm_modulation == FRAME_MOD_MAN) {
        if(frm_length > FRAME_LEN_MAX/2) {
            fprintf(stderr,"Error: Frame length %d is too long for Manchester. Maximum frame length: %d bytes.\n",frm_length,FRAME_LEN_MAX/2);
            return true;
        }
    } else {
        if(frm_length > FRAME_LEN_MAX) {
            fprintf(stderr,"Error: Frame length %d is too long. Maximum frame length: %d bytes.\n",frm_length,FRAME_LEN_MAX);
            return true;
        }
    }
    if(frm_length < FRAME_LEN_MIN) {
        fprintf(stderr,"Error: Frame length %d is too short. Minimum frame length is %d bytes.\n",frm_length,FRAME_LEN_MIN);
        return true;
    }
    // Reserve space for Manchester code in frame
    if(frm_modulation == FRAME_MOD_MAN) {
        frm_length = (frm_length*2)-HEAD_SIZE;
    }
    // Create new empty frame
    current_frame = amframe->NewFrameData(frm_length,frm_modulation);
    current_head = amframe->NewFrameHead(frm_modulation);

    s_resolution = res;
    s_inverse = inv;
    s_average = avg;

    for(int i=0;i<QAMF_LEN_movAvg;i++) {
        qamf_reading.movAvg[i] = 0;
    }
    qamf_reading.sample_count = 0;
    qamf_reading.bufvar = NULL;
    qamf_reading.mu = 0;
    qamf_reading.qsum = 0;
    qamf_reading.xsum = 0;
    qamf_reading.Nvar = 0;
    qamf_reading.bitstart = 0;
    qamf_reading.par = 1;
    qamf_reading.par_alt = 1;
    qamf_reading.bitgrenze = 0;
    qamf_reading.scount = 0;

    return false;
}

/**
 * @brief QAMFrame::InitSSF
 * @param max_msg_size
 * @param max_chunk_size
 * @param max_rs_symbols
 * @return
 */
bool QAMFrame::InitSSF(uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols) {
    uint8_t result_ssf;
    result_ssf = amframe->InitSSF(max_msg_size,max_chunk_size,max_rs_symbols);
    if(result_ssf == 0) {
        ssf_enable = false;
    } else {
        ssf_enable = true;
    }
    return ssf_enable;
}

/**
 * @brief QAMFrame::ReadSample
 * @param databits
 * @param byte
 * @return
 */
int QAMFrame::ReadSample(int byte) {
    int i;

    if(s_average) {
        qamf_reading.movAvg[qamf_reading.sample_count % QAMF_LEN_movAvg] = byte;
        byte = 0;
        for (i = 0; i < QAMF_LEN_movAvg; i++) {
            byte += qamf_reading.movAvg[i];
        }
        byte = (byte+0.5) / QAMF_LEN_movAvg;
    }

    qamf_reading.sample_count++;

    return byte;
}


/**
 * @brief QAMFrame::ReadBit
 * @param bit
 * @param len
 * @param sample_new
 * @return
 */
int QAMFrame::ReadBit(int *bit, int *len, int sample_new) {
    static int sample = 0;
    static int n = 0, y0;
    float l, x1;
    static float x0;


    y0 = sample;
    sample = sample_new;
    qamf_reading.par_alt = qamf_reading.par;
    qamf_reading.par =  (sample >= 0) ? 1 : -1;  // 8bit: 0..127,128..255 (-128..-1,0..127)
    n++;
    if((((qamf_reading.par)*(qamf_reading.par_alt)) > 0)) {
        return 10;
    }

    if (!s_resolution) {
        l = static_cast<float>(n) / s_samples_per_bit;
    }
    else {                                 // more precise bit length measurement
        x1 = sample/(float)(sample-y0);    // helps with low sample rate
        l = (n+x0-x1) / s_samples_per_bit;   // mostly more frames (not always)
        x0 = x1;
    }

    *len = (int)(l+0.5);

    if (!s_inverse){
        *bit = (1+qamf_reading.par_alt)/2;  // top 1, bottom -1
    }
    else {
        *bit = (1-qamf_reading.par_alt)/2;  // sdr # <rev1381?, inverse: bottom 1, top -1
    }

    n = 0;
    return 0;
}

/**
 * @brief QAMFrame::ReadAudioSample
 * @param sample_byte
 * @return
 */
bool QAMFrame::ReadAudioSample(int sample_byte) {
    static int r_sample;
    static int bit, bit_len, bit_status;
    static bool header_found = false;
    static char bitbuf[8];
    static int bit_count = 0;
    static int byte = 0;
    static int byte_count = FRAME_START;

    r_sample = ReadSample(sample_byte);
    bit_status = ReadBit(&bit,&bit_len,r_sample);
    switch (bit_status) {
    case 10:
        //qDebug() << "B_wait";
        break;
    case 0:
        //qDebug() << "B_done";
        //printf("%d",bit);
        //qDebug() << bit << "," << bit_len;
        for (int idx = 0;idx < bit_len;idx++) {
            amframe->IncHeadPos(&current_head);
            current_head.value[current_head.position] = 0x30 + bit;  // Ascii
            if (!header_found) {
                if (amframe->FrameHeadCompare(current_head) >= HEAD_LEN) {
                    header_found = true;
                    printf("Header found\n");
                    fflush(stdout);
                    emit SyncReceived();
                }
            } else {
                bitbuf[bit_count] = bit;
                bit_count++;
                if (bit_count == 8) {
                    bit_count = 0;
                    byte = amframe->Bits2Byte(bitbuf);
                    current_frame.value[byte_count] = byte;
                    byte_count++;
                    if (byte_count == frm_length) {
                        byte_count = FRAME_START;
                        header_found = false;

                        // Link code
                        if(frm_modulation == FRAME_MOD_MAN) {
                            amframe->ManchesterDecode(&current_frame,FRAME_START+1); // Decode Manchester frame
                        } else {
                            amframe->XOR(&current_frame,FRAME_START); // XORing NRZ frame
                        }
                        // Reed-Solomon error correction
                        //if(optsettings.ecc_code != 0) {
                        //    Frame_RSDecode(&frame);
                        //}
                        amframe->PrintFrameData(current_frame,ecc_code);
                        previous_frame = current_frame;
                        emit PacketReceived();
                        fflush(stdout);
                    }
                }
            }
        }
        //fflush(stdout);
        break;
    default:
        break;
    }

    return false;
}

/**
 * @brief QAMFrame::GetLastFrame
 * @return
 */
FrameData QAMFrame::GetLastFrame(void) {
    return previous_frame;
}

/**
 * @brief QAMFrame::DecodeFrame
 * @param frm_dec
 */
void QAMFrame::DecodeFrame(FrameData *frm_dec) {
    // Calculate CRC
    uint16_t crctrs = amframe->GetCRC16(*frm_dec,ecc_code);
    uint16_t crcrec = amframe->CalculateCRC16(frm_dec,ecc_code); // Calculate rewrite internal CRC value

    // RS41 ublox GPS data
    uint16_t year;          // Year, range 1999..2099 (UTC) [- y]
    year = (frm_dec->value[8]) << 24;
    year += (frm_dec->value[9]) << 16;
    year += (frm_dec->value[10]) << 8;
    year += (frm_dec->value[11]) << 0;
    uint8_t month;          // Month, range 1..12 (UTC) [- month]
    month = frm_dec->value[12];
    uint8_t day;            // Day of Month, range 1..31 (UTC) [- d]
    day = frm_dec->value[13];
    uint8_t hour;           // Hour of Day, range 0..23 (UTC) [- h]
    hour = frm_dec->value[14];
    uint8_t min;            // Minute of Hour, range 0..59 (UTC) [- min]
    min = frm_dec->value[15];
    uint8_t sec;            // Seconds of Minute, range 0..59 (UTC) [- s]
    sec = frm_dec->value[16];
    uint8_t gpsFix;         // GPSfix Type
    gpsFix = frm_dec->value[17];
    uint8_t numSV;          // Number of SVs used in Nav Solution
    numSV = frm_dec->value[18];
    int32_t lon;            // Longitude [1e-7 deg]
    lon = (frm_dec->value[19]) << 24;
    lon += (frm_dec->value[20]) << 16;
    lon += (frm_dec->value[21]) << 8;
    lon += (frm_dec->value[22]) << 0;
    double lon_f = ((float)(lon))*1e-7;
    int32_t lat;            // Latitude [1e-7 deg]
    lat = (frm_dec->value[23]) << 24;
    lat += (frm_dec->value[24]) << 16;
    lat += (frm_dec->value[25]) << 8;
    lat += (frm_dec->value[26]) << 0;
    double lat_f = ((float)(lat))*1e-7;
    int32_t hMSL;           // Height above mean sea level [- mm]
    hMSL = (frm_dec->value[27]) << 24;
    hMSL += (frm_dec->value[28]) << 16;
    hMSL += (frm_dec->value[29]) << 8;
    hMSL += (frm_dec->value[30]) << 0;
    float alt_f = ((float)(hMSL))*1e-3;
    int32_t vspeed;         // Down velocity component [- cm/s]
    vspeed = (frm_dec->value[31]) << 24;
    vspeed += (frm_dec->value[32]) << 16;
    vspeed += (frm_dec->value[33]) << 8;
    vspeed += (frm_dec->value[34]) << 0;
    float vspeed_f = ((float)(vspeed))/100;
    uint32_t gSpeed;        // Ground Speed (2-D) [- cm/s]
    gSpeed = (frm_dec->value[35]) << 24;
    gSpeed += (frm_dec->value[36]) << 16;
    gSpeed += (frm_dec->value[37]) << 8;
    gSpeed += (frm_dec->value[38]) << 0;
    float gSpeed_f = ((float)(gSpeed))/100;
    int32_t heading;        // Heading of motion 2-D [1e-5 deg]
    heading = (frm_dec->value[39]) << 24;
    heading += (frm_dec->value[40]) << 16;
    heading += (frm_dec->value[41]) << 8;
    heading += (frm_dec->value[42]) << 0;
    float heading_f = ((float)(heading))*1e-5;
    uint32_t iTOW;          // GPS Millisecond Time of Week [- ms]
    iTOW = (frm_dec->value[43]) << 24;
    iTOW += (frm_dec->value[44]) << 16;
    iTOW += (frm_dec->value[45]) << 8;
    iTOW += (frm_dec->value[46]) << 0;
    // Calculate day
    uint32_t gpstime = iTOW/1000;
    uint32_t gpsday = (gpstime / (24 * 3600)) % 7;
    const char weekday[7][3] = { "Su", "Mo", "Th", "We", "Tr", "Fr", "Sa"};
    int16_t week;           // GPS week (GPS time) [- -]
    week = (frm_dec->value[47]) << 8;
    week += (frm_dec->value[48]) << 0;
    // PTU main temperature only
    uint32_t ptu_main_sensor;
    ptu_main_sensor = (frm_dec->value[49]) << 24;
    ptu_main_sensor += (frm_dec->value[50]) << 16;
    ptu_main_sensor += (frm_dec->value[51]) << 8;
    ptu_main_sensor += (frm_dec->value[52]) << 0;
    // Calculate temperaure
    float ptu_main_sensor_f = (float)(ptu_main_sensor);
    ptu_main_sensor_f = (ptu_main_sensor_f / 100) - 100;
    // Get voltage
    uint8_t bat_voltage = frm_dec->value[53];
    float bat_voltage_f = ((float)(bat_voltage))/10;
    // Sonde ID
    char SondeID[9];
    SondeID[0] = frm_dec->value[54];
    SondeID[1] = frm_dec->value[55];
    SondeID[2] = frm_dec->value[56];
    SondeID[3] = frm_dec->value[57];
    SondeID[4] = frm_dec->value[58];
    SondeID[5] = frm_dec->value[59];
    SondeID[6] = frm_dec->value[60];
    SondeID[7] = frm_dec->value[61];
    SondeID[8] = '\0';
    // Frame count
    uint16_t frame_cnt;
    frame_cnt = (frm_dec->value[62]) << 8;
    frame_cnt += (frm_dec->value[63]);
    // Frequency MHz
    uint16_t freq_mhz = (frm_dec->value[64]) << 8;
    freq_mhz += frm_dec->value[65];
    float freq_mhz_f = ((float)(freq_mhz))/100;
    // Txpower dBm
    uint8_t tx_power = frm_dec->value[66];

    // Check CRC value
    if(crcrec==crctrs) {
        if(numSV == 0) {
            lat_f = 0.0f;
            lon_f = 0.0f;
        }
        // Save to map
        frame_decoded["GPS_YEAR"] = QString::number(year,10);
        frame_decoded["GPS_MONTH"] = QString::number(month,10);
        frame_decoded["GPS_DAY"] = QString::number(day,10);
        frame_decoded["GPS_HOUR"] = QString::number(hour,10);
        frame_decoded["GPS_MINUTE"] = QString::number(min,10);
        frame_decoded["GPS_SECOND"] = QString::number(sec,10);
        frame_decoded["GPS_FIX"] = QString::number(gpsFix,10);
        frame_decoded["GPS_NUMSV"] = QString::number(numSV,10);
        frame_decoded["GPS_LON"] = QString::number(lon_f,'f',6);
        frame_decoded["GPS_LAT"] = QString::number(lat_f,'f',6);
        frame_decoded["GPS_ALT"] = QString::number(alt_f,'f',1);
        frame_decoded["GPS_CLIMBING"] = QString::number(vspeed_f,'f',1);
        frame_decoded["GPS_GROUNDSPEED"] = QString::number(gSpeed_f,'f',1);
        frame_decoded["GPS_HEADING"] = QString::number(heading_f,'f',0);
        frame_decoded["TEMPERATURE_MAIN"] = QString::number(ptu_main_sensor_f,'f',1);
        frame_decoded["SONDE_ID"] = QString(SondeID);
        frame_decoded["FRAME_COUNT"] = QString::number(frame_cnt,10);
        frame_decoded["ONBOARD_VOLTAGE"] = QString::number(bat_voltage_f,'f',1);
        frame_decoded["TX_FREQUENCY"] = QString::number(freq_mhz_f,'f',3);
        frame_decoded["TX_POWER"] = QString::number(tx_power,10);

        // Print UKHAS string
        char ukhas_msg[512];
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
        printf("[CRC FAIL]\n");
    }
    fflush(stdout);
}

/**
 * @brief QAMFrame::GetDecodedFrame
 * @return
 */
QMap<QString, QString> QAMFrame::GetDecodedFrame(void) {
    return frame_decoded;
}
