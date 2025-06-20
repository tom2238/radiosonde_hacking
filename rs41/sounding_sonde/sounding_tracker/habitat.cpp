#include <stdio.h>
#include <curl/curl.h>
#include <string.h>
#include "hb_base64.h"
#include "hb_sha256.h"
#include "habitat.h"



/**
 * @brief Habitat::Habitat
 */
Habitat::Habitat() {

}

/**
 * @brief Habitat::Init
 */
void Habitat::Init(void) {
    curl_global_init(CURL_GLOBAL_ALL);
}

/**
 * @brief Habitat::DeInit
 */
void Habitat::DeInit(void) {
    curl_global_cleanup();
}

/**
 * @brief Habitat::habitat_write_data
 * @param buffer
 * @param size
 * @param nmemb
 * @param userp
 * @return
 */
size_t Habitat::habitat_write_data(void *buffer, size_t size, size_t nmemb, void *userp) {
    //fprintf(stderr,"%s\n", (char *)buffer);
    return size * nmemb;
}

/**
 * @brief Habitat::hash_to_hex
 * @param hash
 * @param line
 */
void Habitat::hash_to_hex(unsigned char *hash, char *line) {
    int idx;

    for (idx=0; idx < 32; idx++)
    {
        sprintf(&(line[idx*2]), "%02x", hash[idx]);
    }
    line[64] = '\0';
}

/**
 * @brief Habitat::write_data
 * @param buffer
 * @param size
 * @param nmemb
 * @param userp
 * @return
 */
size_t Habitat::write_data( void *buffer, size_t size, size_t nmemb, void *userp ) {
    return size * nmemb;
}

/**
 * @brief Habitat::UploadListenerTelemetry
 * @param callsign
 * @param gps_time
 * @param gps_lat
 * @param gps_lon
 * @param radio
 * @param antenna
 */
void Habitat::UploadListenerTelemetry( char *callsign, time_t gps_time, float gps_lat, float gps_lon, char *radio, char *antenna ) {
    char time_string[20];
    struct tm * time_info;

    time_info = localtime (&gps_time);
    strftime(time_string, sizeof(time_string), "%H:%M:%S", time_info);


    CURL *curl;
    CURLcode res;
    char PostFields[300];
    char JsonData[200];

    /* In windows, this will init the winsock stuff */

    /* get a curl handle */
    curl = curl_easy_init(  );
    if ( curl )
    {
        // So that the response to the curl POST doesn;'t mess up my finely crafted display!
        curl_easy_setopt( curl, CURLOPT_WRITEFUNCTION, write_data );

        // Set the URL that is about to receive our POST
        curl_easy_setopt( curl, CURLOPT_URL,
                          "http://habitat.habhub.org/transition/listener_telemetry" );

        // Now specify the POST data
        sprintf( JsonData, "{\"latitude\": %f, \"longitude\": %f}",
                 gps_lat, gps_lon );
        sprintf( PostFields, "callsign=%s&time=%d&data=%s", callsign,
                 (int)gps_time, JsonData );
        curl_easy_setopt( curl, CURLOPT_POSTFIELDS, PostFields );

        // Perform the request, res will get the return code
        res = curl_easy_perform( curl );

        // Check for errors
        if ( res == CURLE_OK )
        {
            printf("Uploaded listener %s %s,%f,%f\n",
                   callsign, time_string, gps_lat, gps_lon );
        }
        else
        {
            printf("curl_easy_perform() failed: %s\n",
                   curl_easy_strerror( res ) );
        }

        // always cleanup
        curl_easy_cleanup( curl );
    }

    /* In windows, this will init the winsock stuff */

    /* get a curl handle */
    curl = curl_easy_init(  );
    if ( curl )
    {
        // So that the response to the curl POST doesn;'t mess up my finely crafted display!
        curl_easy_setopt( curl, CURLOPT_WRITEFUNCTION, write_data );

        // Set the URL that is about to receive our POST
        curl_easy_setopt( curl, CURLOPT_URL,
                          "http://habitat.habhub.org/transition/listener_information" );

        // Now specify the POST data
        sprintf( JsonData, "{\"radio\": \"%s\", \"antenna\": \"%s\"}",
                 radio, antenna );
        sprintf( PostFields, "callsign=%s&time=%d&data=%s", callsign, (int)gps_time, JsonData );
        curl_easy_setopt( curl, CURLOPT_POSTFIELDS, PostFields );

        // Perform the request, res will get the return code
        res = curl_easy_perform( curl );

        // Check for errors
        if ( res != CURLE_OK )
        {
            printf("curl_easy_perform() failed: %s\n",
                   curl_easy_strerror( res ) );
        }

        // always cleanup
        curl_easy_cleanup( curl );
    }
}

/**
 * @brief Habitat::upload_telemetry_packet
 * @param telemetry
 * @param callsign
 */
void Habitat::upload_telemetry_packet(char *telemetry, char *callsign) {
    CURL *curl;
    CURLcode res;
    char curl_error[CURL_ERROR_SIZE];

    /* get a curl handle */
    curl = curl_easy_init();
    if (curl)
    {
        char url[200];
        char base64_data[1000];
        size_t base64_length;
        HB_SHA256::SHA256_CTX ctx;
        unsigned char hash[32];
        char doc_id[100];
        char json[1000], now[32];
        struct curl_slist *headers = NULL;
        time_t rawtime;
        struct tm *tm;
        int retries;
        long int http_res;

        // Get formatted timestamp
        time(&rawtime);
        tm = gmtime(&rawtime);
        strftime(now, sizeof(now), "%Y-%0m-%0dT%H:%M:%SZ", tm);

        // So that the response to the curl PUT doesn't mess up my finely crafted display!
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, habitat_write_data);

        // Set the timeout
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5);

        // RJH capture http errors and report
        // curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1);
        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_error);

        // Avoid curl library bug that happens if above timeout occurs (sigh)
        curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

        // Convert telemetry to base64
        HB_Base64::encode(telemetry, strlen(telemetry), &base64_length, base64_data);
        base64_data[base64_length] = '\0';

        // Take SHA256 hash of the base64 version and express as hex.  This will be the document ID
        HB_SHA256::init(&ctx);
        HB_SHA256::update(&ctx, base64_data, base64_length);
        HB_SHA256::final(&ctx, hash);
        hash_to_hex(hash, doc_id);

        // Create json with the base64 data in hex, the tracker callsign and the current timestamp
        sprintf(json,
                "{\"data\": {\"_raw\": \"%s\"},\"receivers\": {\"%s\": {\"time_created\": \"%s\",\"time_uploaded\": \"%s\"}}}",
                base64_data,
                callsign,
                now,
                now);

        printf("::%s::\n",json);

        // Set the URL that is about to receive our PUT
        sprintf(url, "http://habitat.habhub.org/habitat/_design/payload_telemetry/_update/add_listener/%s", doc_id);

        // Set the headers
        headers = NULL;
        headers = curl_slist_append(headers, "Accept: application/json");
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, "charsets: utf-8");

        // PUT to http://habitat.habhub.org/habitat/_design/payload_telemetry/_update/add_listener/<doc_id> with content-type application/json
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);

        retries = 0;
        do {
            // Perform the request, res will get the return code
            res = curl_easy_perform(curl);

            // Check for errors
            if (res == CURLE_OK)
            {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_res);
                if (http_res != 201 && http_res != 403 && http_res != 409) {
                    fprintf(stderr, "Unexpected HTTP response %ld for URL '%s'\n", http_res, url);
                } else {
                    printf("Send OK\n");
                }
            } else {
                http_res = 0;
                fprintf(stderr, "Failed for URL '%s'\n", url);
                fprintf(stderr ,"curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
                fprintf(stderr, "error: %s\n", curl_error);
            }
        } while ((http_res == 409) && (++retries < 5));

        // always cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    } else {
        printf("Curl error\n");
    }
}

/**
 * @brief Habitat::crc_xmodem_update
 * @param crc
 * @param data
 * @return
 */
uint16_t Habitat::crc_xmodem_update (uint16_t crc, uint8_t data) {
    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for (i=0; i<8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

/**
 * @brief Habitat::ukhas_CRC16_checksum
 * @param string
 * @return
 */
uint16_t Habitat::ukhas_CRC16_checksum (char *string) {
    size_t i;
    uint16_t crc;
    uint8_t c;

    crc = 0xFFFF;

    // Calculate checksum ignoring the first two $s
    for (i = 2; i < strlen(string); i++)
    {
        c = string[i];
        crc = crc_xmodem_update (crc, c);
    }

    return crc;
}
