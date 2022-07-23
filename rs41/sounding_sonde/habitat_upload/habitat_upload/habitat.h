#ifndef HABITAT_H
#define HABITAT_H

#include <time.h>

void Habitat_Init(void);
void Habitat_DeInit(void);
void UploadListenerTelemetry( char *callsign, time_t gps_time, float gps_lat, float gps_lon, char *radio, char *antenna );
void upload_telemetry_packet(char *telemetry, char *callsign);
uint16_t ukhas_CRC16_checksum (char *string);

#endif // HABITAT_H
