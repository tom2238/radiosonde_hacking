#ifndef HB_SHA256_H
#define HB_SHA256_H

#include <stdint.h>

class HB_SHA256 {

public:
    typedef struct {
        uint8_t data[64];
        uint32_t datalen;
        uint32_t bitlen[2];
        uint32_t state[8];
    } SHA256_CTX;
    HB_SHA256();
    static void transform( SHA256_CTX * ctx, uint8_t data[] );
    static void init( SHA256_CTX * ctx );
    static void update( SHA256_CTX * ctx, char data[], uint32_t len );
    static void final( SHA256_CTX * ctx, uint8_t hash[] );
};

#endif // HB_SHA256_H
