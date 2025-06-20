#ifndef HABITAT_H
#define HABITAT_H

#include <time.h>
#include <stdint.h>

class Habitat
{
public:
    Habitat();
    static void Init(void);
    static void DeInit(void);
    static void UploadListenerTelemetry( char *callsign, time_t gps_time, float gps_lat, float gps_lon, char *radio, char *antenna );
    static void upload_telemetry_packet(char *telemetry, char *callsign);
    static uint16_t ukhas_CRC16_checksum (char *string);
private:
    static size_t habitat_write_data(void *buffer, size_t size, size_t nmemb, void *userp);
    static void hash_to_hex(unsigned char *hash, char *line);
    static size_t write_data( void *buffer, size_t size, size_t nmemb, void *userp );
    static uint16_t crc_xmodem_update (uint16_t crc, uint8_t data);
};

#endif // HABITAT_H
