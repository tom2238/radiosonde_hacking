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

#include <stdint.h>
#include "frame.h"

// Private
static int Bits2Byte(char bits[]);
static uint16_t _max_frame_len;
static uint16_t _user_frame_len;
static uint8_t _frame_coding;

/**
 * @brief Frame_Init
 * @param max_frame_len
 * @param user_frame_len
 * @param frame_coding
 * @return
 */
int Frame_Init(uint16_t max_frame_len, uint16_t user_frame_len, uint8_t frame_coding) {
    // Check maximum frame length
    if(max_frame_len > FRAME_LEN_MAX) {
        _max_frame_len = FRAME_LEN_MAX;
    } else {
        _max_frame_len = max_frame_len;
    }
    // Check user frame length
    if(user_frame_len > _max_frame_len) {
        _user_frame_len = _max_frame_len;
    } else {
        _user_frame_len = user_frame_len;
    }
    // Check frame coding
    if(frame_coding == FRAME_MOD_MAN) {
        _frame_coding = FRAME_MOD_MAN;
    } else {
        _frame_coding = FRAME_MOD_NRZ;
    }
    return 0;
}

/**
 * @brief Frame_GetMaxLength
 * @return
 */
uint16_t Frame_GetMaxLength(void) {
    return _max_frame_len;
}

/**
 * @brief Frame_GetUserLength
 * @return
 */
uint16_t Frame_GetUserLength(void) {
    return _user_frame_len;
}

/**
 * @brief Frame_GetCoding
 * @return
 */
uint8_t Frame_GetCoding(void) {
    return _frame_coding;
}

/**
 * @brief Frame_GetCRCSize
 * @return
 */
uint8_t Frame_GetCRCSize(void) {
    return FRAME_CRC_SIZE;
}

/**
 * @brief Frame_GetECCSize
 * @return
 */
uint8_t Frame_GetECCSize(void) {
    return FRAME_ECC_SIZE;
}

/**
 * @brief Frame_GetHeadSize
 * @return
 */
uint8_t Frame_GetHeadSize(void) {
    return FRAME_HEAD_SIZE;
}

// NRZ
//{ 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8} transmitted in frame
//{ 0x86, 0x35, 0xF4, 0x40, 0x93, 0xDF, 0x1A, 0x60} XORed in receiver
// Manchester
//{ 0x9A, 0x99, 0x99, 0x99, 0xA9, 0x6D, 0x55, 0x55} transmitted in frame
// No XORing, scrambling
/**
 * @brief Frame_NewData
 * @param frame_length
 * @param modulation
 * @return
 */
FrameData Frame_NewData(int frame_length, unsigned char modulation) {
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
  for(i=FRAME_HEAD_SIZE;i<newframe.length;i++) {
    newframe.value[i] = 0;
  }
  return newframe;
}

/**
 * @brief Frame_NewHead
 * @param modulation
 * @return
 */
FrameHead Frame_NewHead(unsigned char modulation) {
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
  for(i=0;i<FRAME_HEAD_LEN+1;i++) {
    newhead.value[i] = 'x';
  }
  return newhead;
}

/**
 * @brief Frame_XOR
 * @param frame
 * @param start
 */
void Frame_XOR(FrameData *frame, int start) {
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
 * @brief Frame_CalculateCRC16
 * @param frame
 * @return
 */
uint16_t Frame_CalculateCRC16(FrameData *frame) {
  // CRC-16/CCITT-FALSE
  int crc = 0xFFFF;          // initial value
  int polynomial = 0x1021;   // 0001 0000 0010 0001  (0, 5, 12)
  int i,j;
  uint8_t byte;
  for (i=frame->length-FRAME_CRC_SIZE;i>FRAME_START+1;i--) {
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
  frame->value[frame->length-1] = crc & 0xFF;
  frame->value[frame->length-2] = (crc >> 8) & 0xFF;
  return crc;
}

/**
 * @brief Frame_ManchesterEncode
 * @param frame
 * @param start
 * @return
 */
int Frame_ManchesterEncode(FrameData *frame, int start) {
  int i,j;
  int ManFramePosition = FRAME_START+1;
  int ManBitPosition = 0;
  char Manbitbuf[8];
  uint8_t ManByte;
  uint8_t byte;
  uint8_t frame_bits[8];
  FrameData ManEncode = Frame_NewData(frame->length*2, FRAME_MOD_MAN);
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
  frame->length = (frame->length*2)-FRAME_HEAD_SIZE;
  frame->modulation = FRAME_MOD_MAN;
  for(i=0;i<frame->length;i++){
    frame->value[i] = ManEncode.value[i];
  }
  return 0;
}

/**
 * @brief Bits2Byte
 * @param bits
 * @return
 */
static int Bits2Byte(char bits[]) {
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
