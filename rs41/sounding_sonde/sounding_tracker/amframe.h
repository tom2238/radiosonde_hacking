#ifndef AMFRAME_H
#define AMFRAME_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ssfrs.h"

// Frame and header
#define HEAD_OFS 0 // HEADOFS+HEADLEN <= 64
#define HEAD_LEN 56 // HEADOFS+HEADLEN mod 8 = 0, in bits
#define FRAME_START ((HEAD_OFS+HEAD_LEN)/8)
#define HEAD_SIZE 8 // Head size in bytes

// Data and frame length
#define FRAME_LEN_MAX 1024  // max framelen 1024
#define FRAME_LEN_MIN 18    // HEAD(8) + DATA(8) + ECC(0) + CRC(2)
// Scrambler mask length
#define FRAME_XORMASK_LEN 64
// Default data baud rate
#define DATA_BAUD_RATE 4800
// CRC size in bytes
#define CRC_SIZE 2
// Reed Solomon ECC size in bytes, default value
#define ECC_SIZE 0
// Used frame modulation
#define FRAME_MOD_NRZ 0x1
#define FRAME_MOD_MAN 0x2
// Default frame lenght including HEAD + DATA + ECC + CRC
#define FRAME_DEFAULT_LEN 256

typedef struct {
  uint8_t value[FRAME_LEN_MAX];
  int length;
  unsigned char modulation;
}FrameData;

typedef struct {
  char *header; // NRZ
  char *header_mc; // Manchester2: 01->1,10->0
  char value[HEAD_LEN+1];
  int position;
  unsigned char modulation;
}FrameHead;

class AMFrame
{
public:
    AMFrame(uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols, uint8_t *result_stat);
    ~AMFrame(void);
    FrameData NewFrameData(int frame_length, unsigned char modulation);
    FrameHead NewFrameHead(unsigned char modulation);
    void IncHeadPos(FrameHead *incpos);
    int FrameHeadCompare(FrameHead head);
    void PrintFrameData(FrameData frame, int ecc_size_bytes);
    void PrintFrame_STM32(FrameData frame, int ecc_size_bytes);
    void PrintFrame_RS41GPS(FrameData frame, int ecc_size_bytes);
    void PrintFrame_RS41Sounding(FrameData frame, int ecc_size_bytes);
    void XOR(FrameData *frame, int start);
    void WriteToFile(FrameData frame, FILE *fp, int ecc_size_bytes);
    uint16_t CalculateCRC16(FrameData *frame, int ecc_size_bytes);
    uint16_t crc_xmodem_update (uint16_t crc, uint8_t data);
    uint16_t ukhas_CRC16_checksum (char *string);
    uint16_t GetCRC16(FrameData frame, int ecc_size_bytes);
    int ManchesterEncode(FrameData *frame, int start);
    int ManchesterDecode(FrameData *frame, int start);
    uint8_t CheckRSLimit(uint16_t msg_len, uint16_t parity_len);
    void RSEncode(FrameData *frame);
    void RSDecode(FrameData *frame);

private:
    int Bits2Byte(char bits[]);
    SSFRS *fec_rs_object;
};

#endif // AMFRAME_H
