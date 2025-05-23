#ifndef QAMFRAME_H
#define QAMFRAME_H

#include <QObject>
#include <QAudioFormat>
#include <QDebug>
#include <stdint.h>
#include "amframe.h"

#define QAMF_LEN_movAvg 3

typedef struct {
  int movAvg[QAMF_LEN_movAvg];
  unsigned long sample_count;
  int Nvar;
  float *bufvar;
  float xsum;
  float qsum;
  float mu;
  float bvar[FRAME_LEN_MAX];
  int bitstart;
  int par;
  int par_alt;
  double bitgrenze;
  unsigned long scount;
}QAMF_RBits;

class QAMFrame : public QObject
{
    Q_OBJECT
public:
    explicit QAMFrame(QObject *parent = nullptr);
    ~QAMFrame(void);
    bool Init(bool res, bool inv, bool avg, QAudioFormat audio_format, int baudrate, int frame_length, FrameModulation modulation, int ecc_level);
    bool InitSSF(uint16_t max_msg_size = 1016, uint16_t max_chunk_size = 253, uint16_t max_rs_symbols = 16);
    bool ReadAudioSample(int sample_byte);
    FrameData GetLastFrame(void);
signals:
    void SyncReceived(void);
    void PacketReceived(void);
public slots:

private:
    AMFrame *amframe;
    FrameData current_frame, previous_frame;
    FrameHead current_head;
    // config
    bool s_resolution;
    bool s_inverse;
    bool s_average;
    bool ssf_enable;
    int s_baudrate;
    int frm_length;
    int ecc_code;
    FrameModulation frm_modulation;
    // bits
    QAMF_RBits qamf_reading;
    // wave info
    float s_samples_per_bit;

    // func
    int ReadSample(int byte);
    int ReadBit(int *bit, int *len, int sample_new);
};

#endif // QAMFRAME_H
