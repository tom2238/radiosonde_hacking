#include "qamframe.h"

QAMFrame::QAMFrame(QObject *parent) : QObject(parent) {
    // New frame without SSF
    amframe = new AMFrame();
    ssf_enable = false;
}

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
