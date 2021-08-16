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

#ifndef FRAME_DATA_H
#define FRAME_DATA_H

// Frame and header
#define FRAME_HEAD_OFS 0 // HEADOFS+HEADLEN <= 64
#define FRAME_HEAD_LEN 56 // HEADOFS+HEADLEN mod 8 = 0, in bits
#define FRAME_START ((FRAME_HEAD_OFS+FRAME_HEAD_LEN)/8)
#define FRAME_HEAD_SIZE 8 // Head size in bytes

// Data and frame length
#define FRAME_LEN_MAX 512  // max framelen 150 , user data 140
#define FRAME_USER_LEN 100  // Possible maximums: 251 for MAN, 502 for NRZ
#define FRAME_LEN_MIN 18    // HEAD(8) + DATA(8) + ECC(0) + CRC(2)
// Scrambler mask length
#define FRAME_XORMASK_LEN 64
// CRC size in bytes
#define FRAME_CRC_SIZE 2
// Reed Solomon ECC size in bytes
#define FRAME_ECC_SIZE 0
// Used frame modulation
#define FRAME_MOD_NRZ 0x1
#define FRAME_MOD_MAN 0x2
// Default frame lenght including HEAD + DATA + ECC + CRC
#define FRAME_DEFAULT_LEN 26

typedef struct {
  uint8_t value[FRAME_LEN_MAX];
  int length;
  unsigned char modulation;
}FrameData;

typedef struct {
  char *header; // NRZ
  char *header_mc; // Manchester2: 01->1,10->0
  char value[FRAME_HEAD_LEN+1];
  int position;
  unsigned char modulation;
}FrameHead;

// Functions
// Public
int Frame_Init(uint16_t max_frame_len, uint16_t user_frame_len, uint8_t frame_coding);
uint16_t Frame_GetMaxLength(void);
uint16_t Frame_GetUserLength(void);
uint8_t Frame_GetCoding(void);
uint8_t Frame_GetCRCSize(void);
uint8_t Frame_GetECCSize(void);
uint8_t Frame_GetHeadSize(void);
FrameData Frame_NewData(int frame_length, unsigned char modulation);
FrameHead Frame_NewHead(unsigned char modulation);
void Frame_XOR(FrameData *frame, int start);
uint16_t Frame_CalculateCRC16(FrameData *frame);
int Frame_ManchesterEncode(FrameData *frame, int start);

#endif // FRAME_DATA_H
