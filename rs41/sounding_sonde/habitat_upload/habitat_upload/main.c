#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>
#include "main.h"
#include "habitat.h"

GetOptSettings optsettings = {0.0f,0.0f,"","","","",0};

int main(int argc, char *argv[]) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    uint8_t opt_error = 0;
    int opt = 0;

    // Print UKHAS string
    char ukhas_msg[256];
    char ukhas_msg_crc[256];
    char input_string[256];
    uint16_t ukhas_crc = 0;
    int chars_writed;

    uint16_t packet_counter = 0;

    while ((opt = getopt(argc, argv, "hL:l:C:r:a:c:P:")) != -1){
        switch (opt) {
        case 'h': //Help
            Usage(argv[0]);
            return 0;
            break;
        case 'L': // Latitude
            //strncpy(optsettings.latitude,optarg,sizeof(optsettings.latitude)-1);
            optsettings.latitude = atof(optarg);
            opt_error |= 0x01;
            break;
        case 'l': // Longtitude
            //strncpy(optsettings.outputfile,optarg,sizeof(optsettings.outputfile)-1);
            optsettings.longtitude = atof(optarg);
            opt_error |= 0x02;
            break;
        case 'C': // Callsign
            strncpy(optsettings.callsign,optarg,sizeof(optsettings.callsign)-1);
            opt_error |= 0x04;
            break;
        case 'r': // Radio name
            strncpy(optsettings.radio,optarg,sizeof(optsettings.radio)-1);
            opt_error |= 0x08;
            break;
        case 'a': // Antenna name
            strncpy(optsettings.antenna,optarg,sizeof(optsettings.antenna)-1);
            opt_error |= 0x10;
            break;
        case 'c': // Comment
            strncpy(optsettings.comment,optarg,sizeof(optsettings.comment)-1);
            opt_error |= 0x20;
            break;
        case 'P': // Packet to position ratio
            optsettings.packet_to_position_ratio = atoi(optarg);
            opt_error |= 0x40;
            break;
        case '?': //Unknown option
            //printf("  Error: %c\n", optopt);
            return 1;
        default:
            Usage(argv[0]);
            return 0;
        }
    }

    if(opt_error != 0x7F) {
        fprintf(stderr,"Missing parameter !\n");
        exit(1);
    }



    Habitat_Init();


    UploadListenerTelemetry(optsettings.callsign,time(NULL),optsettings.latitude,optsettings.longtitude,optsettings.radio,optsettings.antenna);

    while(fgets(input_string, sizeof(input_string), stdin)) {
        printf("%s", input_string);
        fflush(stdout);
        if(input_string[0] == '$' && input_string[1] == '$' && input_string[2] != '$') {
            // Remove \n from input
            input_string[strcspn(input_string, "\n")] = 0;
            // Add comment
            chars_writed = snprintf(ukhas_msg,sizeof(ukhas_msg),"%s, %s",input_string,optsettings.comment);
            if(chars_writed < sizeof(ukhas_msg)) {
                // Calculate CRC
                ukhas_crc = ukhas_CRC16_checksum(ukhas_msg);
                // Add CRC
                chars_writed = snprintf(ukhas_msg_crc,sizeof(ukhas_msg_crc),"%s*%04x\n",ukhas_msg,ukhas_crc);
                if(chars_writed < sizeof(ukhas_msg_crc)) {
                    //printf("CRC: %s\n",ukhas_msg_crc);
                    // Send packet
                    upload_telemetry_packet(ukhas_msg_crc,optsettings.callsign);
                    packet_counter++;
                } else {
                    fprintf(stderr,"snprinf buffer length error\n");
                }
            } else {
                fprintf(stderr,"snprinf buffer length error\n");
            }
        } else {
            fprintf(stderr, "upload: string must begin with exactly 2 '$' symbols\n");
        }
        if(packet_counter == optsettings.packet_to_position_ratio) {
            UploadListenerTelemetry(optsettings.callsign,time(NULL),optsettings.latitude,optsettings.longtitude,optsettings.radio,optsettings.antenna);
            packet_counter = 0;
        }
    }

    Habitat_DeInit();

    return 0;
}

void Usage(char *p_name) {
  printf("Habitat uploader\n");
  printf("Usage: %s -L lat -l lon -C callsign -r radio -a antenna -c comment -P ratio| -h\n",p_name);
  printf("  -L <latitude>   Latitude position of station, 48.478645\n");
  printf("  -l <longtitude> Longtitude position of station, -2.568855\n");
  printf("  -C <callsign>   Station callsign\n");
  printf("  -r <radio>      Station radio name\n");
  printf("  -a <antenna>    Station antenna type\n");
  printf("  -c <comment>    Comment section\n");
  printf("  -P <ratio>      Packet to position ratio (send station position every N packets)\n");
  printf("  -h            Show this help\n");
  printf("                Build: %s %s, GCC %s\n", __TIME__, __DATE__, __VERSION__);
}

void SignalHandler(int number) {
   fprintf(stderr,"\nCaught signal %d ... ", number);
   UploadListenerTelemetry(optsettings.callsign,time(NULL),optsettings.latitude,optsettings.longtitude,optsettings.radio,optsettings.antenna);
   Habitat_DeInit();
   fprintf(stderr,"abort\n");
   exit(2);
}
