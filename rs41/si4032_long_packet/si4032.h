#ifndef SI4032_H
#define SI4032_H

// 26 MHz clock from GPS Xtal, float
#define SI4032_XTAL_CLOCK 26.0f

// States
#define SI4032_REGISTER_WRITE 0x80
#define SI4032_WRITE 1
#define SI4032_READ 0

// Registers
#define SI4032_REG_DEVICE_VERSION 0x01
#define SI4032_REG_DEVICE_STATUS 0x02
#define SI4032_REG_INTERRUPT_STATUS_1 0x03
#define SI4032_REG_INTERRUPT_STATUS_2 0x04
#define SI4032_REG_INTERRUPT_ENABLE_1 0x05
#define SI4032_REG_INTERRUPT_ENABLE_2 0x06
#define SI4032_REG_OPERATING_FUNCTION_C1 0x07
#define SI4032_REG_OPERATING_FUNCTION_C2 0x08
#define SI4032_REG_CRYSTAL_LOAD 0x09
#define SI4032_REG_FREQUENCY_OFFSET_1 0x73
#define SI4032_REG_FREQUENCY_OFFSET_2 0x74
#define SI4032_REG_FREQUENCY_BAND_SEL 0x75
#define SI4032_REG_NOMINAL_CARRIER_1 0x76
#define SI4032_REG_NOMINAL_CARRIER_2 0x77
#define SI4032_REG_FREQUENCY_HOPPING_CHAN_SEL 0x79
#define SI4032_REG_FREQUENCY_HOPPING_STEP_SIZE 0x7A
#define SI4032_REG_MODULATION_MODE_C1 0x70
#define SI4032_REG_MODULATION_MODE_C2 0x71
#define SI4032_REG_FREQUENCY_DEVIATION 0x72
#define SI4032_REG_CRYSTAL_OSC_CONTROL_TEST 0x62
#define SI4032_REG_TX_DATA_RATE_1 0x6E
#define SI4032_REG_TX_DATA_RATE_0 0x6F
#define SI4032_REG_TX_POWER 0x6D
#define SI4032_REG_TX_FIFO_C1 0x7C
#define SI4032_REG_TX_FIFO_C2 0x7D
#define SI4032_REG_MCU_OUTPUT_CLOCK 0x0A
#define SI4032_REG_ADC_CONFIG 0x0F
#define SI4032_REG_ADC_SENSOR_OFFSET 0x10
#define SI4032_REG_ADC_VALUE 0x11
#define SI4032_REG_ADC8_CONTROL 0x4F
#define SI4032_REG_PACKET_DATA_CONROL 0x30
#define SI4032_REG_PACKET_EZMAC_STATUS 0x31
#define SI4032_REG_PACKET_HEADER_CONTROL2 0x33
#define SI4032_REG_PACKET_PREAMBLE_LENGTH 0x34
#define SI4032_REG_PACKET_SYNC_WORD3 0x36
#define SI4032_REG_PACKET_SYNC_WORD2 0x37
#define SI4032_REG_PACKET_SYNC_WORD1 0x38
#define SI4032_REG_PACKET_SYNC_WORD0 0x39
#define SI4032_REG_PACKET_TRANS_HEADER3 0x3A
#define SI4032_REG_PACKET_TRANS_HEADER2 0x3B
#define SI4032_REG_PACKET_TRANS_HEADER1 0x3C
#define SI4032_REG_PACKET_TRANS_HEADER0 0x3D
#define SI4032_REG_PACKET_TRANS_PACKET_LEN 0x3E
#define SI4032_REG_TEMPERATURE_SENSOR_CONTROL 0x12
#define SI4032_REG_TEMPERATURE_VALUE_OFFSET 0x13
#define SI4032_REG_LOW_BATTERY_DETECTOR_THS 0x1A
#define SI4032_REG_BATTERY_VOLTAGE_LEVEL 0x1B
#define SI4032_REG_WAKE_UP_TIMER_PERIOD1 0x14
#define SI4032_REG_WAKE_UP_TIMER_PERIOD2 0x15
#define SI4032_REG_WAKE_UP_TIMER_PERIOD3 0x16
#define SI4032_REG_WAKE_UP_TIMER_VALUE1 0x17
#define SI4032_REG_WAKE_UP_TIMER_VALUE2 0x18
#define SI4032_REG_GPIO0_CONFIG 0x0B
#define SI4032_REG_GPIO1_CONFIG 0x0C
#define SI4032_REG_GPIO2_CONFIG 0x0D
#define SI4032_REG_IO_PORT_CONFIG 0x0E
#define SI4032_REG_FIFO_ACCESS 0x7F

// TX power masks
#define SI4032_TX_POWER_1_DBM  0x00
#define SI4032_TX_POWER_2_DBM  0x01
#define SI4032_TX_POWER_5_DBM  0x02
#define SI4032_TX_POWER_8_DBM  0x03
#define SI4032_TX_POWER_11_DBM 0x04
#define SI4032_TX_POWER_14_DBM 0x05
#define SI4032_TX_POWER_17_DBM 0x06
#define SI4032_TX_POWER_20_DBM 0x07

// Tx modulation bits
#define SI4032_TX_MODULATION_SOURCE_DIRECT_GPIO (0x00 << 4)
#define SI4032_TX_MODULATION_SOURCE_DIRECT_SDI (0x01 << 4)
#define SI4032_TX_MODULATION_SOURCE_FIFO (0x02 << 4)
#define SI4032_TX_MODULATION_SOURCE_PN9 (0x03 << 4)
#define SI4032_TX_MODULATION_TYPE_UNMOD (0x00 << 0)
#define SI4032_TX_MODULATION_TYPE_OOK (0x01 << 0)
#define SI4032_TX_MODULATION_TYPE_FSK (0x02 << 0)
#define SI4032_TX_MODULATION_TYPE_GFSK (0x03 << 0)

// Frequency deviation step Hz
#define SI4032_FREQUENCY_DEVIATION_STEP 625
// Frequency offset step Hz
#define SI4032_FREQUENCY_OFFSET_STEP 156.25

// Functions
// Public
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
// FIFO frame test
void Si4032_PacketMode(float data_rate, uint32_t deviation, uint8_t packet_len);
void Si4032_WritePacket(const uint8_t *Data, uint8_t Len);
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

#endif // SI4032_H
