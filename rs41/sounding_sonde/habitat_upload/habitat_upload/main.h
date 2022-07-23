#ifndef MAIN_HAB_H
#define MAIN_HAB_H

typedef struct {
  float latitude;
  float longtitude;
  char callsign[17];
  char radio[17];
  char antenna[17];
  char comment[128];
  int packet_to_position_ratio;
}GetOptSettings;

// Print help and usage of application
void Usage(char *p_name);
// Signal handler
void SignalHandler(int number);

#endif // MAIN_HAB_H
