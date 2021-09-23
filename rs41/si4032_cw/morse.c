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
#include "morse.h"
#include "si4032.h"
#include "init.h"
#include "utils.h"

// Private
static uint16_t Morse_GetChar2Bit(unsigned char alfa);
static uint16_t Morse_GetCharLength(uint16_t value);
static void Morse_WriteCodeChar(uint16_t value, unsigned int wpm);
static void Morse_WriteMorse(unsigned int x,unsigned int len, unsigned int wpm);
static void Morse_WriteDot(unsigned int wpm);
static void Morse_WriteDash(unsigned int wpm);

static uint16_t Morse_GetChar2Bit(unsigned char alfa){
    // unsigned integer 16bits
    // 15 downto 12 'length' = 4bits, 11 downto 0 'data' = 12bits
    // 0111 0000 0000 0000   , 0 . , 1 - | char L (0x4C) .-.. , lengt 0x4, data 0x4, total 0x4004
    //    ^ ^
    // special: lengt=15 space
    // Windows-1250 charset            0x0   , 0x1   , 0x2   , 0x3   , 0x4   , 0x5   , 0x6   , 0x7   , 0x8   , 0x9   , 0xA   , 0xB   , 0xC   , 0xD   , 0xE   , 0xF   ,
    const uint16_t char2bits[256]={0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x601E, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  //0x0
                                       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  //0x1
                                       0xF000, 0x6031, 0x6012, 0x0000, 0x7009, 0x0000, 0x5008, 0x601E, 0x5016, 0x602D, 0x0000, 0x500A, 0x6033, 0x6021, 0x6015, 0x5012,  //0x2
                                       0x501F, 0x500F, 0x5007, 0x5003, 0x5001, 0x5000, 0x5010, 0x5018, 0x501C, 0x501E, 0x6038, 0x602A, 0x0000, 0x5011, 0x0000, 0x600C,  //0x3
                                       0x601A, 0x2001, 0x4008, 0x400A, 0x3004, 0x1000, 0x4002, 0x3006, 0x4000, 0x2000, 0x4007, 0x3005, 0x4004, 0x2003, 0x2002, 0x3007,  //0x4
                                       0x4006, 0x400D, 0x3002, 0x3000, 0x1001, 0x3001, 0x4001, 0x3003, 0x4009, 0x400B, 0x400C, 0x0000, 0x0000, 0x0000, 0x0000, 0x600D,  //0x5
                                       0x0000, 0x2001, 0x4008, 0x400A, 0x3004, 0x1000, 0x4002, 0x3006, 0x4000, 0x2000, 0x4007, 0x3005, 0x4004, 0x2003, 0x2002, 0x3007,  //0x6
                                       0x4006, 0x400D, 0x3002, 0x3000, 0x1001, 0x3001, 0x4001, 0x3003, 0x4009, 0x400B, 0x400C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  //0x7
                                       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3000, 0x0000, 0x3000, 0x1001, 0x400C, 0x400C,  //0x8
                                       0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3000, 0x0000, 0x3000, 0x1001, 0x400C, 0x400C,  //0x9
                                       0x0000, 0x0000, 0x0000, 0x4004, 0x0000, 0x2001, 0x0000, 0x0000, 0x0000, 0x0000, 0x3000, 0x0000, 0x0000, 0x0000, 0x0000, 0x400C,  //0xA
                                       0x0000, 0x0000, 0x0000, 0x4004, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2001, 0x3000, 0x0000, 0x4004, 0x0000, 0x4004, 0x400C,  //0xB
                                       0x3002, 0x2001, 0x2001, 0x2001, 0x2001, 0x4004, 0x400A, 0x400A, 0x400A, 0x1000, 0x1000, 0x1000, 0x1000, 0x2000, 0x2000, 0x3004,  //0xC
                                       0x3004, 0x2002, 0x2002, 0x3007, 0x3007, 0x3007, 0x3007, 0x0000, 0x3002, 0x3001, 0x3001, 0x3001, 0x3001, 0x400B, 0x1001, 0x0000,  //0xD
                                       0x3002, 0x2001, 0x2001, 0x2001, 0x2001, 0x4004, 0x400A, 0x400A, 0x400A, 0x1000, 0x1000, 0x1000, 0x1000, 0x2000, 0x2000, 0x3004,  //0xE
                                       0x3004, 0x2002, 0x2002, 0x3007, 0x3007, 0x3007, 0x3007, 0x0000, 0x3002, 0x3001, 0x3001, 0x3001, 0x3001, 0x400B, 0x1001, 0x0000}; //0xF

    return char2bits[(unsigned short int)alfa];
    // ^^^^^^^^^^^^ It is magical? :-)  How is it stored there? :-)
}

static uint16_t Morse_GetCharLength(uint16_t value){
   return value >> 12;
}

static void Morse_WriteCodeChar(uint16_t value, unsigned int wpm){
    uint16_t lengt=0;
    uint16_t morse=0;
    lengt = value >> 12;
    morse = value & 0x0FFF;
    if(lengt==15) { //space
        delay(7*1200/wpm); // 7 dots space
    }
    else if(lengt==0) { //ignore this
    }
    else {
        Morse_WriteMorse(morse, lengt, wpm);
    }
}

static void Morse_WriteMorse(unsigned int x,unsigned int len, unsigned int wpm) {
    unsigned i;
    if(len==0){len=1;}
    for (i = 1 << (len-1); i > 0; i = i / 2)  {
        if(x & i){
            Morse_WriteDash(wpm);
        }
        else{
            Morse_WriteDot(wpm);
        }
        if(i!=1) {
            delay(1200/wpm); // 1 dot space
        }
    }
}

static void Morse_WriteDot(unsigned int wpm) {
    unsigned int tdur = 1200/wpm; //dot length in ms
    Si4032_EnableTx();
    delay(tdur);
    Si4032_DisableTx();
}

static void Morse_WriteDash(unsigned int wpm) {
    unsigned int tdur = 3*1200/wpm; //dash length in ms
    Si4032_EnableTx();
    delay(tdur);
    Si4032_DisableTx();
}

void Morse_SendMessage(char *message, unsigned int wpm) {
    int i = 0;
    while(message[i]!='\0'){
        if(Morse_GetCharLength(Morse_GetChar2Bit(message[i]))==0){ //skip not know characters
            i++;
            continue;
        }
        Morse_WriteCodeChar(Morse_GetChar2Bit(message[i]), wpm);
        if(!((message[i]==' ') | (message[i+1]==' ' && message[i]!=' '))) {
            delay(3*1200/wpm); // Space, 3 dots
        }
        i++;
    }
}
