/* --------------------------------------------------------------------------------------------- */
/* Small System Framework                                                                        */
/*                                                                                               */
/* ssfrs.c                                                                                       */
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

#include "ssfrs.h"

#define SSF_ASSERT(x) do { if (!(x)) SSFPortAssert(__FILE__, (uint32_t) __LINE__); } while (0)
#define SSF_REQUIRE(x) SSF_ASSERT(x)
#define SSF_ENSURE(x) SSF_ASSERT(x)
#define SSF_ERROR() SSF_ASSERT(0)

#define SSF_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define SSF_MIN(x, y) (((x) < (y)) ? (x) : (y))

#define GF_ADD(x, y) (((uint8_t) x) ^ ((uint8_t) y))
#define GF_SUB(x, y) GF_ADD(x, y)
#define GF_MUL(x, y) ((x == 0) || (y == 0) ? 0 : _gfExp[_gfLog[x] + _gfLog[y]])
#define MOD255(m) ((((m) >> 8) + ((m) & 0xFF)) < 255 ? (((m) >> 8) + ((m) & 0xFF)) : ((((m) >> 8) + ((m) & 0xFF)) < (2 * 255)) ? ((((m) >> 8) + ((m) & 0xFF)) - 255): (((m) >> 8) + ((m) & 0xFF)) - (2 * 255))
#define GF_DIV(x, y) ((x == 0) ? 0 : _gfExp[MOD255(_gfLog[x] + 255 - _gfLog[y])])
#define GF_POW(x, p) (_gfExp[MOD255(_gfLog[x] * p)])
#define GF_INV(x) (_gfExp[255 - _gfLog[x]])


SSFRS::SSFRS(uint16_t max_msg_size, uint16_t max_chunk_size, uint16_t max_rs_symbols, uint8_t *result_stat) {
    uint8_t result = 0x00;
        /* The maximum total size in bytes of a message to be encoded or decoded */
        if(max_msg_size > FRAME_SSF_RS_MAX_MESSAGE_SIZE) {
            // Warning.
            _max_msg_size = FRAME_SSF_RS_MAX_MESSAGE_SIZE;
            result += FRAME_SSF_ERROR_MAX_MSG_SIZE;
        } else {
            _max_msg_size = max_msg_size;
        }
        /* The maximum number of bytes that will be encoded with up to SSF_RS_MAX_SYMBOLS bytes */
        if(max_chunk_size > FRAME_SSF_RS_MAX_CHUNK_SIZE) {
            // Warning.
            _max_chunk_size = FRAME_SSF_RS_MAX_CHUNK_SIZE;
            result += FRAME_SSF_ERROR_MAX_CHUNK_SIZE;
        } else {
            _max_chunk_size = max_chunk_size;
        }
        /* The maximum number of chunks that a message will be broken up into for encoding and decoding */
        if((_max_msg_size % _max_chunk_size) == 0) {
            _max_number_of_chunks = (_max_msg_size / _max_chunk_size);
        } else {
            _max_number_of_chunks = (_max_msg_size / _max_chunk_size) + 1;
        }
        /* The maximum number of symbols in bytes that will encode up to SSF_RS_MAX_CHUNK_SIZE bytes */
        /* Reed-Solomon can correct SSF_RS_MAX_SYMBOLS/2 bytes with errors in a message */
        if((max_rs_symbols < FRAME_SSF_RS_MIN_SYMBOLS) || (max_rs_symbols > FRAME_SSF_RS_MAX_SYMBOLS)) {
            // Warning.
            _max_rs_size = FRAME_SSF_RS_DEFAULT_SIZE;
            result += FRAME_SSF_ERROR_MAX_RS_SIZE;
        } else {
            _max_rs_size = max_rs_symbols;
        }
        /* For now we are limiting the total of chunk bytes and symbols to 254 max */
        if((_max_chunk_size + _max_rs_size) > FRAME_SSF_RS_TOTAL_CHUNK_LIMIT) {
            // Warning.
            result += FRAME_SSF_ERROR_CHUNK_LIMIT;
        }
        /* For now we are limiting the total encoded msg + ecc to 1024 bytes */
        if((_max_rs_size * _max_number_of_chunks) + _max_msg_size > FRAME_SSF_RS_TOTAL_LIMIT) {
            // Warning.
            result += FRAME_SSF_ERROR_TOTAL_LIMIT;
        }
        if(result == 0) {
            _ssfrs_initialized = true;
        } else {
            _ssfrs_initialized = false;
        }
        //printf("CS:%d,MSG:%d,CN:%d,RS:%d\n",_max_chunk_size,_max_msg_size,_max_number_of_chunks,_max_rs_size);
        *result_stat = result;
}

/* --------------------------------------------------------------------------------------------- */
/* Make a copy of a GF polynomial.                                                               */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_GFPolyCopy(const GFPoly_t* src, GFPoly_t* dst) {
    SSF_REQUIRE(src != NULL);
    SSF_REQUIRE(dst != NULL);
    memcpy(dst, src, sizeof(GFPoly_t));
}

#if SSF_RS_ENABLE_DECODING == 1
/* --------------------------------------------------------------------------------------------- */
/* Make a reversed copy of a GF polynomial.                                                      */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_GFPolyCopyRev(const GFPoly_t* src, GFPoly_t* dst) {
    uint16_t i;

    SSF_REQUIRE(src != NULL);
    SSF_REQUIRE(dst != NULL);

    for (i = 0; i < src->len; i++) {
        dst->array[src->len - i - 1] = src->array[i];
    }
    dst->len = src->len;
}

/* --------------------------------------------------------------------------------------------- */
/* Scale a GF polynomial.                                                                        */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_GFPolyScale(const GFPoly_t *p, uint8_t s, GFPoly_t* d) {
    size_t len;
    const uint8_t* pptr;
    uint8_t* dptr;

    SSF_REQUIRE(p != NULL);
    SSF_REQUIRE(d != NULL);

    len = d->len = p->len;
    pptr = p->array;
    dptr = d->array;

    while (len) {
        *dptr = GF_MUL(*pptr, s);
        dptr++;
        pptr++;
        len--;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* Add two polynomials.                                                                          */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_GFPolyAdd(const GFPoly_t *p, const GFPoly_t *q, GFPoly_t *d) {
    uint16_t i;

    SSF_REQUIRE(p != NULL);
    SSF_REQUIRE(q != NULL);
    SSF_REQUIRE(d != NULL);

    /* Initialize d with zeros */
    d->len = SSF_MAX(p->len, q->len);
    memset(d->array, 0, d->len);

    /* Copy p into new array */
    for (i = 0; i < p->len; i++) {
        d->array[i + d->len - p->len] = p->array[i];
    }

    /* Add q to new array */
    for (i = 0; i < q->len; i++) {
        d->array[i + d->len - q->len] ^= q->array[i];
    }
}

/* --------------------------------------------------------------------------------------------- */
/* Returns the evaluation of the polynomial.                                                     */
/* --------------------------------------------------------------------------------------------- */
uint8_t SSFRS::_GFPolyEval(const GFPoly_t* p, uint8_t x) {
    uint16_t i;
    uint8_t y;

    SSF_REQUIRE(p != NULL);

    y = p->array[0];
    for (i = 1; i < p->len; i++) {
        y = GF_MUL(y, x) ^ p->array[i];
    }
    return y;
}
#endif /* SSF_RS_ENABLE_DECODING */

/* --------------------------------------------------------------------------------------------- */
/* Multiply two polynomials.                                                                     */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_GFPolyMul(const GFPoly_t *p, const GFPoly_t *q, GFPoly_t *d) {
    uint16_t j;
    uint8_t i;

    SSF_REQUIRE(p != NULL);
    SSF_REQUIRE(q != NULL);
    SSF_REQUIRE(d != NULL);

    /* Initialize d with zeros */
    d->len = p->len + q->len - 1;
    SSF_ASSERT(d->len <= sizeof(d->array));
    memset(d->array, 0, d->len);

    for (j = 0; j < q->len; j++) {
        for (i = 0; i < p->len; i++) {
            d->array[i + j] ^= GF_MUL(p->array[i], q->array[j]);
        }
    }
}

/* --------------------------------------------------------------------------------------------- */
/* Divide two polynomials.                                                                       */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_GFPolyDiv(const GFPoly_t *dividend, const GFPoly_t *divisor, GFPoly_t *quotient,
                       GFPoly_t *remainder) {
    uint8_t coef;
    GFPoly_t msg_out;
    uint16_t i;
    uint16_t j;

    _GFPolyCopy(dividend, &msg_out);

    for (i = 0; i <= (dividend->len - divisor->len); i++) {
        coef = msg_out.array[i];
        if (coef != 0) {
            for (j = 1; j < divisor->len; j++) {
                if (divisor->array[j] != 0) {
                    msg_out.array[i + j] ^= GF_MUL(divisor->array[j], coef);
                }
            }
        }
    }

    quotient->len = msg_out.len - (divisor->len - 1);
    for (i = 0; i < (divisor->len - 1); i++) {
        remainder->array[i] = msg_out.array[i + quotient->len];
    }
    remainder->len = (divisor->len - 1);
}

/* --------------------------------------------------------------------------------------------- */
/* Reed-Solomon Galois Field GF(2^8) Encoding Operations                                         */
/* --------------------------------------------------------------------------------------------- */
#if SSF_RS_ENABLE_ENCODING == 1
/* --------------------------------------------------------------------------------------------- */
/* Build Reed-Solomon generator polynomial                                                       */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_RSGeneratorPoly(uint8_t nsym, GFPoly_t *g) {
    uint16_t i;
    GFPoly_t c;
    GFPoly_t q;

    SSF_REQUIRE(g != NULL);

    g->len = 1;
    g->array[0] = 1;
    q.len = 2;
    q.array[0] = 1;

    for (i = 0; i < nsym; i++) {
        _GFPolyCopy(g, &c);
        q.array[1] = GF_POW(2, i);
        _GFPolyMul(&c, &q, g);
    }
}

/* --------------------------------------------------------------------------------------------- */
/* Encodes a message by returning the "checksum" to attach to the message                        */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_RSEncodeMsg(GFPoly_t *msg_in, uint8_t nsym, GFPoly_t *msg_out) {
    uint16_t i;
    GFPoly_t gen;
    GFPoly_t quot;

    _RSGeneratorPoly(nsym, &gen);

    for (i = 0; i < gen.len - 1; i++) {
        msg_in->array[msg_in->len + i] = 0;
    }
    msg_in->len += gen.len - 1;

    _GFPolyDiv(msg_in, &gen, &quot, msg_out);
}
#endif /* SSF_RS_ENABLE_ENCODING */

/* --------------------------------------------------------------------------------------------- */
/* Reed-Solomon Galois Field GF(2^8) Decoding Operations                                         */
/* --------------------------------------------------------------------------------------------- */
#if SSF_RS_ENABLE_DECODING == 1
/* --------------------------------------------------------------------------------------------- */
/* Calculate syndromes.                                                                          */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_RSCalcSyndromes(const GFPoly_t *msg, uint8_t nsym, GFPoly_t *synd) {
    uint16_t i;

    memset(synd->array, 0, ((size_t) nsym) + 1);
    synd->len = ((size_t) nsym) + 1;

    for (i = 0; i < nsym; i++) {
        synd->array[i + 1] = _GFPolyEval(msg, GF_POW(2, i));
    }
}
/* --------------------------------------------------------------------------------------------- */
/* Returns true if message syndromes match, else false.                                          */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::_RSCheck(const GFPoly_t *msg, uint8_t nsym) {
    GFPoly_t synd;
    uint16_t i;

    _RSCalcSyndromes(msg, nsym, &synd);
    for(i = 1; i <= nsym; i++) {
        if (synd.array[i] != 0) return false;
    }
    return true;
}

/* --------------------------------------------------------------------------------------------- */
/* Returns true if syndromes match, else false.                                                  */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::_RSCheckSynd(const GFPoly_t* msg, uint8_t nsym, GFPoly_t *synd) {
    uint16_t i;

    _RSCalcSyndromes(msg, nsym, synd);
    for (i = 1; i <= nsym; i++) {
        if (synd->array[i] != 0) return false;
    }
    return true;
}

/* --------------------------------------------------------------------------------------------- */
/* Evaluates for errors.                                                                         */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_RSFindErrorEvaluator(GFPoly_t *synd, GFPoly_t *err_loc, uint8_t nsym,
                                 GFPoly_t *remainder) {
    GFPoly_t tmp;
    GFPoly_t tmp2;
    GFPoly_t quotient;

    _GFPolyMul(synd, err_loc, &tmp);
    tmp2.len = ((size_t) nsym) + 2;
    tmp2.array[0] = 1;
    memset(&tmp2.array[1], 0, tmp2.len - 1);
    _GFPolyDiv(&tmp, &tmp2, &quotient, remainder);
}

/* --------------------------------------------------------------------------------------------- */
/* Returns true if errors located and fixed, else false.                                         */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::_RSFindErrorLocator(const GFPoly_t *synd, uint8_t nsym, GFPoly_t *err_loc) {
    GFPoly_t old_loc;
    GFPoly_t new_loc;
    GFPoly_t tmp;
    GFPoly_t tmp2;
    size_t synd_shift;
    uint16_t i;
    uint16_t j;
    uint16_t K;
    uint8_t delta;
    size_t errs;

    err_loc->array[0] = 1;
    err_loc->len = 1;
    old_loc.array[0] = 1;
    old_loc.len = 1;

    synd_shift = synd->len - nsym;

    for (i = 0; i < nsym; i++) {
        K = (uint16_t) (i + synd_shift);

        delta = synd->array[K];
        for (j = 1; j < err_loc->len; j++) {
            delta ^= GF_MUL(err_loc->array[err_loc->len - 1 - j], synd->array[K - j]);
        }

        /* Shift polynomials to compute the next degree */
        old_loc.array[old_loc.len] = 0;
        old_loc.len++;

        /* Iteratively estimate the errata locator and evaluator polynomials */
        if (delta != 0) {
            if (old_loc.len > err_loc->len) {
                _GFPolyScale(&old_loc, delta, &new_loc);
                _GFPolyScale(err_loc, GF_INV(delta), &old_loc);
                _GFPolyCopy(&new_loc, err_loc);
            }
            /* Update with the discrepancy */
            _GFPolyScale(&old_loc, delta, &tmp);
            _GFPolyCopy(err_loc, &tmp2);
            _GFPolyAdd(&tmp2, &tmp, err_loc);
        }
    }

    /* Check if the result is correct, that there's not too many errors to correct */
    while ((err_loc->len) && (err_loc->array[0] == 0)) {
        err_loc->len--;
        memmove(&err_loc->array[0], &err_loc->array[1], err_loc->len);
    }
    errs = err_loc->len - 1;
    if ((errs * 2) > nsym) {
        /* Too many errors to correct */
        return false;
    }
    return true;
}

/* --------------------------------------------------------------------------------------------- */
/* Returns true if number of errors are correctable, else false.                                 */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::_RSFindErrors(const GFPoly_t *err_loc, uint8_t nmess, GFPoly_t *err_pos) {
    uint8_t errs = (uint8_t) (err_loc->len - 1);
    uint16_t i;

    err_pos->len = 0;

    for (i = 0; i < nmess; i++) {
        if (_GFPolyEval(err_loc, GF_POW(2, i)) == 0) {
            err_pos->array[err_pos->len] = (uint8_t) (nmess - 1 - i);
            err_pos->len++;
        }
    }

    if (err_pos->len != errs) {
        /* Too many (or few) errors found by Chien Search for the errata locator polynomial */
        return false;
    }
    return true;
}

/* --------------------------------------------------------------------------------------------- */
/* Finds the location of the errors.                                                             */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_RSFindErrataLocator(const GFPoly_t *e_pos, GFPoly_t *e_loc) {
    GFPoly_t tmp;
    GFPoly_t tmp2;
    GFPoly_t tmp3;
    GFPoly_t one;
    uint16_t i;

    e_loc->array[0] = 1;
    e_loc->len = 1;

    one.array[0] = 1;
    one.len = 1;

    for (i = 0; i < e_pos->len; i++) {
        tmp3.array[0] = GF_POW(2, e_pos->array[i]);
        tmp3.array[1] = 0;
        tmp3.len = 2;
        _GFPolyAdd(&one, &tmp3, &tmp);
        _GFPolyMul(e_loc, &tmp, &tmp2);
        _GFPolyCopy(&tmp2, e_loc);
    }
}

/* --------------------------------------------------------------------------------------------- */
/* Corrects errata.                                                                              */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::_RSCorrectErrata(GFPoly_t *msg_in, GFPoly_t *synd, GFPoly_t *err_pos, GFPoly_t* msg_out) {
    GFPoly_t coef_pos;
    GFPoly_t err_loc;
    GFPoly_t synd_rev;
    GFPoly_t err_eval;
    GFPoly_t err_eval_rev;
    GFPoly_t X;
    GFPoly_t E;
    GFPoly_t err_loc_prime_tmp;
    uint16_t i;
    uint16_t j;
    uint8_t l;
    size_t Xlength;
    uint8_t Xi_inv;
    uint8_t err_loc_prime;
    uint8_t y;
    uint8_t magnitude;

    for (i = 0; i < err_pos->len; i++) {
        coef_pos.array[i] = (uint8_t) (msg_in->len - 1 - err_pos->array[i]);
    }
    coef_pos.len = err_pos->len;

    _RSFindErrataLocator(&coef_pos, &err_loc);

    _GFPolyCopyRev(synd, &synd_rev);

    _RSFindErrorEvaluator(&synd_rev, &err_loc, (uint8_t) (err_loc.len - 1), &err_eval_rev);
    _GFPolyCopyRev(&err_eval_rev, &err_eval);

    X.len = 0;
    for (i = 0; i < coef_pos.len; i++) {
        l = 255 - coef_pos.array[i];
        X.array[i] = GF_POW(2, -l);
        X.len++;
    }

    memset(E.array, 0, msg_in->len);
    E.len = msg_in->len;
    Xlength = X.len;
    for (i = 0; i < X.len; i++) {
        Xi_inv = GF_INV(X.array[i]);

        err_loc_prime_tmp.len = 0;

        for (j = 0; j < Xlength; j++) {
            if (j != i) {
                err_loc_prime_tmp.array[err_loc_prime_tmp.len] = GF_SUB(1, GF_MUL(Xi_inv, X.array[j]));
                err_loc_prime_tmp.len++;
            }
        }
        err_loc_prime = 1;

        for (j = 0; j < err_loc_prime_tmp.len; j++) {
            err_loc_prime = GF_MUL(err_loc_prime, err_loc_prime_tmp.array[j]);
        }

        _GFPolyCopyRev(&err_eval, &err_eval_rev);

        y = _GFPolyEval(&err_eval_rev, Xi_inv);
        y = GF_MUL(GF_POW(X.array[i], 1), y);

        if (err_loc_prime == 0) {
            return false;
        }
        magnitude = GF_DIV(y, err_loc_prime);
        E.array[err_pos->array[i]] = magnitude;
    }

    _GFPolyAdd(msg_in, &E, msg_out);
    return true;
}

/* --------------------------------------------------------------------------------------------- */
/* Computes Forney syndromes.                                                                    */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::_RSForneySyndromes(const GFPoly_t *synd, GFPoly_t *fsynd) {
    uint16_t i;

    for (i = 0; i < synd->len; i++) {
        fsynd->array[i] = synd->array[i + 1];
    }
    fsynd->len = synd->len - 1;
}

/* --------------------------------------------------------------------------------------------- */
/* Corrects a message.                                                                           */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::_RSCorrectMsg(GFPoly_t *msg_in, uint8_t nsym, GFPoly_t *msg_out) {
    GFPoly_t err_loc;
    GFPoly_t err_loc_rev;
    GFPoly_t synd;
    GFPoly_t fsynd;
    GFPoly_t tmp;
    GFPoly_t err_pos;

    if (msg_in->len > 255) {
        return false; // Message is too long
    }
    _GFPolyCopy(msg_in, msg_out);

    if (_RSCheckSynd(msg_in, nsym, &synd)) {
        /* No Errors */
        return true;
    }

    _RSForneySyndromes(&synd, &fsynd);
    _RSFindErrorLocator(&fsynd, nsym, &err_loc);
    _GFPolyCopyRev(&err_loc, &err_loc_rev);
    _RSFindErrors(&err_loc_rev, (uint8_t) msg_out->len, &err_pos);
    if (err_pos.len == 0) {
        /* Could not locate error */
        return false;
    }

    _GFPolyCopy(msg_out, &tmp);
    _RSCorrectErrata(&tmp, &synd, &err_pos, msg_out);

    return _RSCheck(msg_out, nsym);
}
#endif /* SSF_RS_ENABLE_DECODING */

#if SSF_RS_ENABLE_ENCODING == 1
/* --------------------------------------------------------------------------------------------- */
/* Encodes msg as a series of 1 or more eccNumBytes sized ECC blocks contiguously in eccBuf.     */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::Encode(const uint8_t *msg, uint16_t msgLen, uint8_t *eccBuf, uint16_t eccBufSize,
                 uint16_t *eccBufLen, uint8_t eccNumBytes, uint8_t chunkSize) {
    GFPoly_t chunk;
    GFPoly_t ecc;

    SSF_REQUIRE(msg != NULL);
    SSF_REQUIRE(msgLen > 0);
    SSF_REQUIRE(eccBuf != NULL);
    SSF_REQUIRE(eccBufLen != NULL);
    SSF_REQUIRE((uint16_t)eccNumBytes <= _max_rs_size);
    SSF_REQUIRE(eccNumBytes > 0);
    SSF_REQUIRE((uint16_t)chunkSize <= _max_chunk_size);
    SSF_REQUIRE(chunkSize > 0);

    /* Iterate over msg and encode chunks up to chunkSize with eccChunkBytes symbols */
    *eccBufLen = 0;
    while (msgLen) {
        SSF_ASSERT(eccBufSize >= eccNumBytes);

        /* Last chunk? If yes update chunkSize */
        if (msgLen < chunkSize) chunkSize = (uint8_t) msgLen;

        /* Encode */
        memcpy(chunk.array, msg, chunkSize);
        chunk.len = chunkSize;
        _RSEncodeMsg(&chunk, eccNumBytes, &ecc);

        /* Advance to next message chunk */
        msgLen -= chunkSize;
        msg += chunkSize;

        /* Save ECC and advance to next ecc block */
        SSF_ASSERT(ecc.len == eccNumBytes);
        memcpy(eccBuf, ecc.array, eccNumBytes);
        eccBufSize -= eccNumBytes;
        eccBuf += eccNumBytes;
        *eccBufLen += eccNumBytes;
    }
}
#endif /* SSF_RS_ENABLE_ENCODING */

#if SSF_RS_ENABLE_DECODING == 1
/* --------------------------------------------------------------------------------------------- */
/* Returns true and msgLen if decoding was successful, else false.                               */
/* --------------------------------------------------------------------------------------------- */
bool SSFRS::Decode(uint8_t* msg, uint16_t msgSize, uint16_t *msgLen, uint8_t eccNumBytes,
                 uint8_t chunkSize) {
    uint16_t tmpSize;
    uint8_t tmpChunkSize;
    uint16_t eccBlockBytes;
    uint8_t *ecc;
    GFPoly_t chunkIn;
    GFPoly_t chunkOut;

    SSF_REQUIRE(msg != NULL);
    SSF_REQUIRE(msgSize > 0);
    SSF_REQUIRE(msgLen != NULL);
    SSF_REQUIRE((uint16_t)eccNumBytes <= _max_rs_size);
    SSF_REQUIRE(eccNumBytes > 0);
    SSF_REQUIRE((uint16_t)chunkSize <= _max_chunk_size);
    SSF_REQUIRE(chunkSize > 0);

    /* Determine start of the ECC blocks */
    eccBlockBytes = 0;
    tmpSize = msgSize;
    tmpChunkSize = chunkSize;
    while (tmpSize) {
        /* If remaining message too small return false */
        if (tmpSize <= eccNumBytes) return false;

        /* Last chunk? If yes update chunkSize */
        if ((tmpSize - eccNumBytes) < tmpChunkSize)
        { tmpChunkSize = (uint8_t) (tmpSize - eccNumBytes); }

        tmpSize -= eccNumBytes;
        tmpSize -= tmpChunkSize;

        eccBlockBytes += eccNumBytes;
    }
    ecc = &msg[msgSize - eccBlockBytes];

    /* Decode msg chunk by chunk correcting the msg buffer along the way */
    msgSize -= eccBlockBytes;
    *msgLen = 0;
    while (msgSize) {
        /* Last chunk? If yes update chunkSize */
        if (msgSize < chunkSize) chunkSize = (uint8_t) msgSize;

        /* Attempt correction, if it fails return false */
        memcpy(chunkIn.array, msg, chunkSize);
        memcpy(&chunkIn.array[chunkSize], ecc, eccNumBytes);
        chunkIn.len = ((size_t) chunkSize) + eccNumBytes;
        if (_RSCorrectMsg(&chunkIn, eccNumBytes, &chunkOut) == false)
        { return false; }
        SSF_ASSERT(chunkOut.len == (((size_t)chunkSize) + eccNumBytes));
        memcpy(msg, chunkOut.array, chunkSize);

        /* Advance to next msg block */
        msg += chunkSize;
        msgSize -= chunkSize;
        *msgLen += chunkSize;

        /* Advance to next ECC block */
        ecc += eccNumBytes;
    }

    /* All chunks were successfully corrected */
    return true;
}
#endif /* SSF_RS_ENABLE_DECODING */

/* --------------------------------------------------------------------------------------------- */
/* Never returns (except for unit testing), reports assertion failure.                           */
/* --------------------------------------------------------------------------------------------- */
void SSFRS::SSFPortAssert(const char* file, unsigned int line) {
    {fprintf(stderr,"SSF Assertion: %s:%u\r\n", file, line); }
    //for (;;);
}

/**
 * @brief SSFRS_GetMsgSize
 * @return
 */
uint16_t SSFRS::GetMsgSize(void) {
    return _max_msg_size;
}

/**
 * @brief SSFRS_GetRSSize
 * @return
 */
uint16_t SSFRS::GetRSSize(void) {
    return _max_rs_size;
}

/**
 * @brief SSFRS_GetMaxChunks
 * @return
 */
uint16_t SSFRS::GetMaxChunks(void) {
    return _max_number_of_chunks;
}

/**
 * @brief SSFRS_GetChunkSize
 * @return
 */
uint16_t SSFRS::GetChunkSize(void) {
    return _max_chunk_size;
}
