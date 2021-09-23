# RS41 libraries
## si4032
```
void Si4032_SetFrequency(const float freq_in_mhz);
uint8_t Si4032_GetVersion(void);
void Si4032_DisableTx(void);
void Si4032_EnableTx(void);
void Si4032_InhibitTx(void);
void Si4032_PacketTx(void);
void Si4032_SetTxPower(uint8_t power);
void Si4032_SetTxDataRate(uint32_t rate_bps);
void Si4032_SetModulatioSource(uint8_t source);
void Si4032_SetModulatioType(uint8_t type);
void Si4032_SetFrequencyDeviation(uint32_t frequency_hz);
void Si4032_SetFrequencyOffset(uint16_t offset_hz);
int8_t Si4032_GetTemperature(void);
uint16_t Si4032_GetBatteryVoltage(void);
// FIFO packet
void Si4032_PacketMode(enum SI4032_PACKET_TYPE packet_type, uint32_t data_rate, uint32_t deviation, uint8_t packet_len, uint8_t preamble_length);
void Si4032_WriteShortPacket(const uint8_t *Data, uint8_t Len);
void Si4032_WritePacketData(const uint8_t *Data, uint16_t start, uint16_t size);
uint8_t Si4032_IsFIFOEmpty(void);
uint8_t Si4032_IsPacketSent(void);
void Si4032_EnablePacketSentInterrupt(void);
void Si4032_EnableFIFOEmptyInterrupt(void);
void Si4032_ClearInterruptStatus(void);
void Si4032_ClearFIFO(void);
// Init
void Si4032_Init(void);
void Si4032_Init2(void);
// Registers debuging
void Si4032_PrintRegisters(void);
```

## frame
```
int Frame_Init(uint16_t max_frame_len, uint16_t user_frame_len, uint8_t frame_coding);
uint16_t Frame_GetMaxLength(void);
uint16_t Frame_GetUserLength(void);
uint8_t Frame_GetCoding(void);
uint8_t Frame_GetCRCSize(void);
uint8_t Frame_GetECCSize(void);
uint8_t Frame_GetHeadSize(void);
FrameData Frame_NewData(int frame_length, unsigned char modulation);
FrameHead Frame_NewHead(unsigned char modulation);
void Frame_XOR(FrameData *frame, int start);
uint16_t Frame_CalculateCRC16(FrameData *frame);
int Frame_ManchesterEncode(FrameData *frame, int start);
```

## utils
```
extern char* itoa( int value, char *string, int radix ) ;
extern char* ltoa( long value, char *string, int radix ) ;
extern char* utoa( unsigned long value, char *string, int radix ) ;
extern char* ultoa( unsigned long value, char *string, int radix ) ;
void console_putc(char c);
char console_getc(int wait);
void console_puts(char *s);
int console_gets(char *s, int len);
void console_print_int(int number);
void console_print_float(float number);
void ftoa(float n, char* res, int afterpoint);
uint64_t millis(void);
void delay(uint64_t duration);
```

## ublox6
```
uint8_t Ublox6_Init(uBlox6_init_state init_state);
int Ublox6_HandleByte(uint8_t data);
void Ublox6_GetLastData(uBlox6_GPSData *gpsEntry);
void Ublox6_Poll(uint8_t msgClass, uint8_t msgID);
```


