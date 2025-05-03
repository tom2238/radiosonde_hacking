/* --------------------------------------------------------------------------------------------- */
/* Small System Framework                                                                        */
/*                                                                                               */
/* ssfrs.h                                                                                       */
/* Provides Reed-Solomon FEC encoder/decoder interface.                                          */
/*                                                                                               */
/* Limitations:                                                                                  */
/*     Decode interface does not support erasure corrections, only error corrections.            */
/*     Eraseure corrections are only useful if the location of an error is known.                */
/*                                                                                               */
/* Reed-Solomon algorithms and code inspired and adapted from:                                   */
/*     https://en.wikiversity.org/wiki/Reed-Solomon_codes_for_coders                             */
/*                                                                                               */
/* MOD255 macro algorithm inspired and adapted from:                                             */
/*     http://homepage.cs.uiowa.edu/~jones/bcd/mod.shtml                                         */
/*                                                                                               */
/* BSD-3-Clause License                                                                          */
/* Copyright 2021 Supurloop Software LLC                                                         */
/*                                                                                               */
/* Redistribution and use in source and binary forms, with or without modification, are          */
/* permitted provided that the following conditions are met:                                     */
/*                                                                                               */
/* 1. Redistributions of source code must retain the above copyright notice, this list of        */
/* conditions and the following disclaimer.                                                      */
/* 2. Redistributions in binary form must reproduce the above copyright notice, this list of     */
/* conditions and the following disclaimer in the documentation and/or other materials provided  */
/* with the distribution.                                                                        */
/* 3. Neither the name of the copyright holder nor the names of its contributors may be used to  */
/* endorse or promote products derived from this software without specific prior written         */
/* permission.                                                                                   */
/*                                                                                               */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   */
/* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    */
/* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL      */
/* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE */
/* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    */
/* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  */
/* OF THE POSSIBILITY OF SUCH DAMAGE.                                                            */
/* --------------------------------------------------------------------------------------------- */
#ifndef SSF_RS_INCLUDE_H
#define SSF_RS_INCLUDE_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------------------------- */
/* Configure ssfrs's Reed-Solomon interface                                                      */
/* --------------------------------------------------------------------------------------------- */
/* The maximum total size in bytes of a message to be encoded or decoded */
#define FRAME_SSF_RS_MAX_MESSAGE_SIZE 1016
/* The maximum number of bytes that will be encoded with up to SSF_RS_MAX_SYMBOLS bytes */
#define FRAME_SSF_RS_MAX_CHUNK_SIZE 253
/* The maximum number of symbols in bytes that will encode up to SSF_RS_MAX_CHUNK_SIZE bytes */
/* Reed-Solomon can correct SSF_RS_MAX_SYMBOLS/2 bytes with errors in a message */
#define FRAME_SSF_RS_MAX_SYMBOLS 253
#define FRAME_SSF_RS_MIN_SYMBOLS 2
#define FRAME_SSF_RS_DEFAULT_SIZE FRAME_SSF_RS_MIN_SYMBOLS
/* For now we are limiting the total of chunk bytes and symbols to 255 max */
/* 2^8-1 , n^m-1 */
#define FRAME_SSF_RS_TOTAL_CHUNK_LIMIT 255
/* For now we are limiting the total encoded msg + ecc to 1024 bytes */
#define FRAME_SSF_RS_TOTAL_LIMIT 1024
/* Error codes */
#define FRAME_SSF_ERROR_MAX_MSG_SIZE 0x01
#define FRAME_SSF_ERROR_MAX_CHUNK_SIZE 0x02
#define FRAME_SSF_ERROR_MAX_RS_SIZE 0x04
#define FRAME_SSF_ERROR_CHUNK_LIMIT 0x08
#define FRAME_SSF_ERROR_TOTAL_LIMIT 0x10

/* 1 to enable Reed-Solomon encoding interface, else 0. */
#define SSF_RS_ENABLE_ENCODING (1u)

/* 1 to enable Reed-Solomon decoding interface, else 0. */
#define SSF_RS_ENABLE_DECODING (1u)

/* The maximum total size in bytes of a message to be encoded or decoded */
#define SSF_RS_MAX_MESSAGE_SIZE (223)
#if SSF_RS_MAX_MESSAGE_SIZE > 2048
#error SSFRS invalid SSF_RS_MAX_MESSAGE_SIZE.
#endif

/* The maximum number of bytes that will be encoded with up to SSF_RS_MAX_SYMBOLS bytes */
#define SSF_RS_MAX_CHUNK_SIZE (231u)
#if SSF_RS_MAX_CHUNK_SIZE > 253
#error SSFRS invalid SSF_RS_MAX_CHUNK_SIZE.
#endif

/* The maximum number of chunks that a message will be broken up into for encoding and decoding */
#if SSF_RS_MAX_MESSAGE_SIZE % SSF_RS_MAX_CHUNK_SIZE == 0
#define SSF_RS_MAX_CHUNKS (SSF_RS_MAX_MESSAGE_SIZE / SSF_RS_MAX_CHUNK_SIZE)
#else
#define SSF_RS_MAX_CHUNKS ((SSF_RS_MAX_MESSAGE_SIZE / SSF_RS_MAX_CHUNK_SIZE) + 1)
#endif

/* The maximum number of symbols in bytes that will encode up to SSF_RS_MAX_CHUNK_SIZE bytes */
/* Reed-Solomon can correct SSF_RS_MAX_SYMBOLS/2 bytes with errors in a message */
#define SSF_RS_MAX_SYMBOLS (24ul)
#if (SSF_RS_MAX_SYMBOLS < 2) || (SSF_RS_MAX_SYMBOLS > 254)
#error SSFRS Invalid SSF_RS_MAX_SYMBOLS.
#endif

/* For now we are limiting the total of chunk bytes and symbols to 255 max */
#if SSF_RS_MAX_CHUNK_SIZE + SSF_RS_MAX_SYMBOLS > 255
#error SSFRS total of SSF_RS_MAX_CHUNK_SIZE + SSF_RS_MAX_SYMBOLS not supported.
#endif

/* For now we are limiting the total of chunk bytes and symbols to 255 max */
#if FRAME_SSF_RS_TOTAL_CHUNK_LIMIT > 255
#error SSFRS total size of chunk over limit, for GF(2^8) is 255 max block size
#endif

/* For now we are limiting the total encoded msg + ecc to 61440 (60KiB) */
#if (SSF_RS_MAX_SYMBOLS * SSF_RS_MAX_CHUNKS) + SSF_RS_MAX_MESSAGE_SIZE > 61440ul
#error SSFRS total of SSF_RS_MAX_CHUNK_SIZE + SSF_RS_MAX_SYMBOLS not supported.
#endif

/* --------------------------------------------------------------------------------------------- */
/* Galois Field GF(2^8) Base Polynomial Operations                                               */
/* --------------------------------------------------------------------------------------------- */
typedef struct GFPoly {
    uint8_t array[FRAME_SSF_RS_TOTAL_CHUNK_LIMIT];
    size_t len;
} GFPoly_t;

class SSFRS
{
public:
    SSFRS(uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols, uint8_t *result_stat);
#if SSF_RS_ENABLE_ENCODING == 1
    void _RSEncodeMsg(GFPoly_t *msg_in, uint8_t nsym, GFPoly_t *msg_out);
    void Encode(const uint8_t *msg, uint16_t msgLen, uint8_t *eccBuf, uint16_t eccBufSize, uint16_t *eccBufLen, uint8_t eccNumBytes, uint8_t chunkSize);
#endif /* SSF_RS_ENABLE_ENCODING */
#if SSF_RS_ENABLE_DECODING == 1
    bool Decode(uint8_t* msg, uint16_t msgSize, uint16_t *msgLen, uint8_t eccNumBytes, uint8_t chunkSize);
#endif /* SSF_RS_ENABLE_DECODING */
    uint16_t GetMsgSize(void);
    uint16_t GetRSSize(void);
    uint16_t GetMaxChunks(void);
    uint16_t GetChunkSize(void);

private:
    void SSFPortAssert(const char* file, unsigned int line);
    void _GFPolyCopy(const GFPoly_t* src, GFPoly_t* dst);
    void _GFPolyCopyRev(const GFPoly_t* src, GFPoly_t* dst);
    void _GFPolyScale(const GFPoly_t *p, uint8_t s, GFPoly_t* d);
    void _GFPolyAdd(const GFPoly_t *p, const GFPoly_t *q, GFPoly_t *d);
    uint8_t _GFPolyEval(const GFPoly_t* p, uint8_t x);
    void _GFPolyMul(const GFPoly_t *p, const GFPoly_t *q, GFPoly_t *d);
    void _GFPolyDiv(const GFPoly_t *dividend, const GFPoly_t *divisor, GFPoly_t *quotient, GFPoly_t *remainder);
#if SSF_RS_ENABLE_ENCODING == 1
    void _RSGeneratorPoly(uint8_t nsym, GFPoly_t *g);
#endif /* SSF_RS_ENABLE_ENCODING */
#if SSF_RS_ENABLE_DECODING == 1
    void _RSCalcSyndromes(const GFPoly_t *msg, uint8_t nsym, GFPoly_t *synd);
    bool _RSCheck(const GFPoly_t *msg, uint8_t nsym);
    bool _RSCheckSynd(const GFPoly_t* msg, uint8_t nsym, GFPoly_t *synd);
    void _RSFindErrorEvaluator(GFPoly_t *synd, GFPoly_t *err_loc, uint8_t nsym, GFPoly_t *remainder);
    bool _RSFindErrorLocator(const GFPoly_t *synd, uint8_t nsym, GFPoly_t *err_loc);
    bool _RSFindErrors(const GFPoly_t *err_loc, uint8_t nmess, GFPoly_t *err_pos);
    void _RSFindErrataLocator(const GFPoly_t *e_pos, GFPoly_t *e_loc);
    bool _RSCorrectErrata(GFPoly_t *msg_in, GFPoly_t *synd, GFPoly_t *err_pos, GFPoly_t* msg_out);
    void _RSForneySyndromes(const GFPoly_t *synd, GFPoly_t *fsynd);
    bool _RSCorrectMsg(GFPoly_t *msg_in, uint8_t nsym, GFPoly_t *msg_out);
#endif /* SSF_RS_ENABLE_DECODING */

    void *ssfUnused;

    // SSF Reed-Solomon FEC configuration
    uint16_t _max_msg_size;
    uint16_t _max_chunk_size;
    uint16_t _max_rs_size;
    uint16_t _max_number_of_chunks;
    bool _ssfrs_initialized = false;

    /* --------------------------------------------------------------------------------------------- */
    /* Galois Field GF(2^8) Base Operations */
    /* --------------------------------------------------------------------------------------------- */
    /* GF(2^8) logorithm lookup table */
    const uint8_t _gfLog[256] = {
        0x00, 0x00, 0x01, 0x19, 0x02, 0x32, 0x1A, 0xC6, 0x03, 0xDF, 0x33, 0xEE, 0x1B, 0x68, 0xC7, 0x4B,
        0x04, 0x64, 0xE0, 0x0E, 0x34, 0x8D, 0xEF, 0x81, 0x1C, 0xC1, 0x69, 0xF8, 0xC8, 0x08, 0x4C, 0x71,
        0x05, 0x8A, 0x65, 0x2F, 0xE1, 0x24, 0x0F, 0x21, 0x35, 0x93, 0x8E, 0xDA, 0xF0, 0x12, 0x82, 0x45,
        0x1D, 0xB5, 0xC2, 0x7D, 0x6A, 0x27, 0xF9, 0xB9, 0xC9, 0x9A, 0x09, 0x78, 0x4D, 0xE4, 0x72, 0xA6,
        0x06, 0xBF, 0x8B, 0x62, 0x66, 0xDD, 0x30, 0xFD, 0xE2, 0x98, 0x25, 0xB3, 0x10, 0x91, 0x22, 0x88,
        0x36, 0xD0, 0x94, 0xCE, 0x8F, 0x96, 0xDB, 0xBD, 0xF1, 0xD2, 0x13, 0x5C, 0x83, 0x38, 0x46, 0x40,
        0x1E, 0x42, 0xB6, 0xA3, 0xC3, 0x48, 0x7E, 0x6E, 0x6B, 0x3A, 0x28, 0x54, 0xFA, 0x85, 0xBA, 0x3D,
        0xCA, 0x5E, 0x9B, 0x9F, 0x0A, 0x15, 0x79, 0x2B, 0x4E, 0xD4, 0xE5, 0xAC, 0x73, 0xF3, 0xA7, 0x57,
        0x07, 0x70, 0xC0, 0xF7, 0x8C, 0x80, 0x63, 0x0D, 0x67, 0x4A, 0xDE, 0xED, 0x31, 0xC5, 0xFE, 0x18,
        0xE3, 0xA5, 0x99, 0x77, 0x26, 0xB8, 0xB4, 0x7C, 0x11, 0x44, 0x92, 0xD9, 0x23, 0x20, 0x89, 0x2E,
        0x37, 0x3F, 0xD1, 0x5B, 0x95, 0xBC, 0xCF, 0xCD, 0x90, 0x87, 0x97, 0xB2, 0xDC, 0xFC, 0xBE, 0x61,
        0xF2, 0x56, 0xD3, 0xAB, 0x14, 0x2A, 0x5D, 0x9E, 0x84, 0x3C, 0x39, 0x53, 0x47, 0x6D, 0x41, 0xA2,
        0x1F, 0x2D, 0x43, 0xD8, 0xB7, 0x7B, 0xA4, 0x76, 0xC4, 0x17, 0x49, 0xEC, 0x7F, 0x0C, 0x6F, 0xF6,
        0x6C, 0xA1, 0x3B, 0x52, 0x29, 0x9D, 0x55, 0xAA, 0xFB, 0x60, 0x86, 0xB1, 0xBB, 0xCC, 0x3E, 0x5A,
        0xCB, 0x59, 0x5F, 0xB0, 0x9C, 0xA9, 0xA0, 0x51, 0x0B, 0xF5, 0x16, 0xEB, 0x7A, 0x75, 0x2C, 0xD7,
        0x4F, 0xAE, 0xD5, 0xE9, 0xE6, 0xE7, 0xAD, 0xE8, 0x74, 0xD6, 0xF4, 0xEA, 0xA8, 0x50, 0x58, 0xAF
    };

    /* GF(2^8) exponent lookup table */
    const uint8_t _gfExp[512] = {
        0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1D, 0x3A, 0x74, 0xE8, 0xCD, 0x87, 0x13, 0x26,
        0x4C, 0x98, 0x2D, 0x5A, 0xB4, 0x75, 0xEA, 0xC9, 0x8F, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0,
        0x9D, 0x27, 0x4E, 0x9C, 0x25, 0x4A, 0x94, 0x35, 0x6A, 0xD4, 0xB5, 0x77, 0xEE, 0xC1, 0x9F, 0x23,
        0x46, 0x8C, 0x05, 0x0A, 0x14, 0x28, 0x50, 0xA0, 0x5D, 0xBA, 0x69, 0xD2, 0xB9, 0x6F, 0xDE, 0xA1,
        0x5F, 0xBE, 0x61, 0xC2, 0x99, 0x2F, 0x5E, 0xBC, 0x65, 0xCA, 0x89, 0x0F, 0x1E, 0x3C, 0x78, 0xF0,
        0xFD, 0xE7, 0xD3, 0xBB, 0x6B, 0xD6, 0xB1, 0x7F, 0xFE, 0xE1, 0xDF, 0xA3, 0x5B, 0xB6, 0x71, 0xE2,
        0xD9, 0xAF, 0x43, 0x86, 0x11, 0x22, 0x44, 0x88, 0x0D, 0x1A, 0x34, 0x68, 0xD0, 0xBD, 0x67, 0xCE,
        0x81, 0x1F, 0x3E, 0x7C, 0xF8, 0xED, 0xC7, 0x93, 0x3B, 0x76, 0xEC, 0xC5, 0x97, 0x33, 0x66, 0xCC,
        0x85, 0x17, 0x2E, 0x5C, 0xB8, 0x6D, 0xDA, 0xA9, 0x4F, 0x9E, 0x21, 0x42, 0x84, 0x15, 0x2A, 0x54,
        0xA8, 0x4D, 0x9A, 0x29, 0x52, 0xA4, 0x55, 0xAA, 0x49, 0x92, 0x39, 0x72, 0xE4, 0xD5, 0xB7, 0x73,
        0xE6, 0xD1, 0xBF, 0x63, 0xC6, 0x91, 0x3F, 0x7E, 0xFC, 0xE5, 0xD7, 0xB3, 0x7B, 0xF6, 0xF1, 0xFF,
        0xE3, 0xDB, 0xAB, 0x4B, 0x96, 0x31, 0x62, 0xC4, 0x95, 0x37, 0x6E, 0xDC, 0xA5, 0x57, 0xAE, 0x41,
        0x82, 0x19, 0x32, 0x64, 0xC8, 0x8D, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xDD, 0xA7, 0x53, 0xA6,
        0x51, 0xA2, 0x59, 0xB2, 0x79, 0xF2, 0xF9, 0xEF, 0xC3, 0x9B, 0x2B, 0x56, 0xAC, 0x45, 0x8A, 0x09,
        0x12, 0x24, 0x48, 0x90, 0x3D, 0x7A, 0xF4, 0xF5, 0xF7, 0xF3, 0xFB, 0xEB, 0xCB, 0x8B, 0x0B, 0x16,
        0x2C, 0x58, 0xB0, 0x7D, 0xFA, 0xE9, 0xCF, 0x83, 0x1B, 0x36, 0x6C, 0xD8, 0xAD, 0x47, 0x8E, 0x01,
        0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1D, 0x3A, 0x74, 0xE8, 0xCD, 0x87, 0x13, 0x26, 0x4C,
        0x98, 0x2D, 0x5A, 0xB4, 0x75, 0xEA, 0xC9, 0x8F, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x9D,
        0x27, 0x4E, 0x9C, 0x25, 0x4A, 0x94, 0x35, 0x6A, 0xD4, 0xB5, 0x77, 0xEE, 0xC1, 0x9F, 0x23, 0x46,
        0x8C, 0x05, 0x0A, 0x14, 0x28, 0x50, 0xA0, 0x5D, 0xBA, 0x69, 0xD2, 0xB9, 0x6F, 0xDE, 0xA1, 0x5F,
        0xBE, 0x61, 0xC2, 0x99, 0x2F, 0x5E, 0xBC, 0x65, 0xCA, 0x89, 0x0F, 0x1E, 0x3C, 0x78, 0xF0, 0xFD,
        0xE7, 0xD3, 0xBB, 0x6B, 0xD6, 0xB1, 0x7F, 0xFE, 0xE1, 0xDF, 0xA3, 0x5B, 0xB6, 0x71, 0xE2, 0xD9,
        0xAF, 0x43, 0x86, 0x11, 0x22, 0x44, 0x88, 0x0D, 0x1A, 0x34, 0x68, 0xD0, 0xBD, 0x67, 0xCE, 0x81,
        0x1F, 0x3E, 0x7C, 0xF8, 0xED, 0xC7, 0x93, 0x3B, 0x76, 0xEC, 0xC5, 0x97, 0x33, 0x66, 0xCC, 0x85,
        0x17, 0x2E, 0x5C, 0xB8, 0x6D, 0xDA, 0xA9, 0x4F, 0x9E, 0x21, 0x42, 0x84, 0x15, 0x2A, 0x54, 0xA8,
        0x4D, 0x9A, 0x29, 0x52, 0xA4, 0x55, 0xAA, 0x49, 0x92, 0x39, 0x72, 0xE4, 0xD5, 0xB7, 0x73, 0xE6,
        0xD1, 0xBF, 0x63, 0xC6, 0x91, 0x3F, 0x7E, 0xFC, 0xE5, 0xD7, 0xB3, 0x7B, 0xF6, 0xF1, 0xFF, 0xE3,
        0xDB, 0xAB, 0x4B, 0x96, 0x31, 0x62, 0xC4, 0x95, 0x37, 0x6E, 0xDC, 0xA5, 0x57, 0xAE, 0x41, 0x82,
        0x19, 0x32, 0x64, 0xC8, 0x8D, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xDD, 0xA7, 0x53, 0xA6, 0x51,
        0xA2, 0x59, 0xB2, 0x79, 0xF2, 0xF9, 0xEF, 0xC3, 0x9B, 0x2B, 0x56, 0xAC, 0x45, 0x8A, 0x09, 0x12,
        0x24, 0x48, 0x90, 0x3D, 0x7A, 0xF4, 0xF5, 0xF7, 0xF3, 0xFB, 0xEB, 0xCB, 0x8B, 0x0B, 0x16, 0x2C,
        0x58, 0xB0, 0x7D, 0xFA, 0xE9, 0xCF, 0x83, 0x1B, 0x36, 0x6C, 0xD8, 0xAD, 0x47, 0x8E, 0x01, 0x02
    };
};

#endif /* SSF_RS_INCLUDE_H */
