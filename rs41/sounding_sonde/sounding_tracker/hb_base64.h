#ifndef HB_BASE64_H
#define HB_BASE64_H

#include <stdint.h>
#include <stdlib.h>

class HB_Base64
{
public:
    HB_Base64();
    static char *encode(const char *data, size_t input_length, size_t * output_length, char *encoded_data);
    static char *decode(const char *data, size_t input_length, size_t * output_length);

private:
    static void build_decoding_table();
    static void cleanup();
};

#endif // HB_BASE64_H
