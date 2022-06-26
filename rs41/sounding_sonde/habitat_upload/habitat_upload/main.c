#include <curl/curl.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "base64.h"
#include "sha256.h"

char callsign[]="TOM2238";

size_t habitat_write_data(void *buffer, size_t size, size_t nmemb, void *userp)
{
    //fprintf(stderr,"%s\n", (char *)buffer);
    return size * nmemb;
}

void hash_to_hex(unsigned char *hash, char *line)
{
    int idx;

    for (idx=0; idx < 32; idx++)
    {
        sprintf(&(line[idx*2]), "%02x", hash[idx]);
    }
    line[64] = '\0';
}

void upload_telemetry_packet(char *telemetry)
{
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
        SHA256_CTX ctx;
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
        base64_encode(telemetry, strlen(telemetry), &base64_length, base64_data);
        base64_data[base64_length] = '\0';

        // Take SHA256 hash of the base64 version and express as hex.  This will be the document ID
        sha256_init(&ctx);
        sha256_update(&ctx, base64_data, base64_length);
        sha256_final(&ctx, hash);
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
            }
            else
            {
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

int main()
{
    char s[200];

    curl_global_init(CURL_GLOBAL_ALL);

    while(fgets(s, sizeof(s), stdin))
    {
        printf("%s", s);
        fflush(stdout);
        if(s[0] == '$' && s[1] == '$' && s[2] != '$')
        {
            upload_telemetry_packet(s);
        }
        else
        {
            fprintf(stderr, "upload: string must begin with exactly 2 '$' symbols\n");
        }
    }

    curl_global_cleanup();

    return 0;
}
