// Global LibOpenCM3 libraries
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/spi.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "si4032.h"
#include "itoa.h"
#include "uart.h"

// Private
static inline uint8_t Si4032_Register(const uint8_t register_addr, uint8_t value, uint8_t write);
static uint8_t Si4032_SpiTransaction(uint16_t data);
static inline uint8_t Si4032_Write(uint8_t reg, uint8_t value);
static inline uint8_t Si4032_Read(uint8_t reg);
static void Si4032_SoftReset(void);
static uint8_t Si4032_SetRegisterBits(uint8_t reg, uint8_t mask);
static uint8_t Si4032_ClearRegisterBits(uint8_t reg, uint8_t mask);
static uint8_t Si4032_AssignRegisterBits(uint8_t reg, uint8_t mask, uint8_t bits);
static void Si4032_Print(int reg, uint8_t value);

// Interrupt status
static uint8_t _ItStatus1;
static uint8_t _ItStatus2;

static inline uint8_t Si4032_Register(const uint8_t register_addr, uint8_t value, uint8_t write) {
    return Si4032_SpiTransaction(((write ? register_addr | SI4032_REGISTER_WRITE : register_addr) << 8) | value);
}

static uint8_t Si4032_SpiTransaction(uint16_t data) {
    // Clear NSEL pin
    gpio_clear(SI4032_NSEL_GPIO,SI4032_NSEL_PIN);
    // Blocking SPI write
    spi_send(SI4032_SPI_PORT,data);
    // Blocking SPI read
    uint8_t read_val = (uint8_t)spi_read(SI4032_SPI_PORT);
    // Set NSEL pin
    gpio_set(SI4032_NSEL_GPIO,SI4032_NSEL_PIN);
    return read_val;
}

static inline uint8_t Si4032_Write(uint8_t reg, uint8_t value) {
    return Si4032_SpiTransaction(((reg | SI4032_REGISTER_WRITE) << 8U) | value);
}

static inline uint8_t Si4032_Read(uint8_t reg) {
    return Si4032_SpiTransaction((reg << 8U) | 0xFFU);
}

void Si4032_SetFrequency(const float freq_in_mhz) {
    uint8_t hbsel = (uint8_t) ((freq_in_mhz * (30.0f / SI4032_XTAL_CLOCK)) >= 480.0f ? 1 : 0);
    uint8_t fb = (uint8_t) ((((uint8_t)((freq_in_mhz * (30.0f / SI4032_XTAL_CLOCK)) / 10) - 24) - (24 * hbsel)) / (1 + hbsel));
    uint8_t gen_div  =  3;  // constant - not possible to change!
    uint16_t fc = (uint16_t) (((freq_in_mhz / ((SI4032_XTAL_CLOCK / gen_div) * (hbsel + 1))) - fb - 24) * 64000);
    Si4032_Write(SI4032_REG_FREQUENCY_BAND_SEL, (uint8_t) (0b01000000 | (fb & 0b11111) | ((hbsel & 0b1) << 5)));
    Si4032_Write(SI4032_REG_NOMINAL_CARRIER_1, (uint8_t) (((uint16_t)fc >> 8) & 0xff));
    Si4032_Write(SI4032_REG_NOMINAL_CARRIER_2, (uint8_t) ((uint16_t)fc & 0xff));
}

uint8_t Si4032_GetVersion(void) {
    return Si4032_Read(SI4032_REG_DEVICE_VERSION);
}

void Si4032_DisableTx(void) {
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x40);
}

static void Si4032_SoftReset(void) {
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x80);
}

void Si4032_EnableTx(void) {
    // Modified to set the PLL and Crystal enable bits to high. Not sure if this makes much differents.
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x4B);
}

void Si4032_InhibitTx(void) {
    // Sleep mode, but with PLL idle mode enabled, in an attempt to reduce drift on key-up.
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x43);
}

void Si4032_SetTxPower(uint8_t power) {
    Si4032_Write(SI4032_REG_TX_POWER, (power & 0x07));
}

void Si4032_SetTxDataRate(uint32_t rate_bps) {
    // Check limits
    if(rate_bps > 256000) { // Rate is to hight
        rate_bps = 256000;
    }
    if(rate_bps < 123) { // Rate is to low
        rate_bps = 123;
    }
    // Because Si4032 in RS41 use 26 MHz clock
    // recalculate correct value
    float rate_bps_cal = ((float)(rate_bps)*15)/13;
    // If rate is under 30 kbps, set txdtrtscale bit to 1
    uint16_t rate_reg;
    // txdr[15:0] = DR_TX(kbps) * 2^(16+5*txdtrtscale) / 1 MHz
    if(rate_bps_cal < 30000) {
        rate_reg = (rate_bps_cal*(2097152)/1000000); // 2^21
        Si4032_SetRegisterBits(SI4032_REG_MODULATION_MODE_C1,0b00100000);
    } else {
        rate_reg = (rate_bps_cal*(65536)/1000000); // 2^16
        Si4032_ClearRegisterBits(SI4032_REG_MODULATION_MODE_C1,0b00100000);
    }
    // Set higher bits
    Si4032_Write(SI4032_REG_TX_DATA_RATE_1,(rate_reg >> 8) & 0xFF);
    // Set lower bits
    Si4032_Write(SI4032_REG_TX_DATA_RATE_0,(rate_reg >> 0) & 0xFF);
}

void Si4032_SetModulatioSource(uint8_t source) {
    Si4032_AssignRegisterBits(SI4032_REG_MODULATION_MODE_C2, 0b00110000, source);
}

void Si4032_SetModulatioType(uint8_t type) {
    Si4032_AssignRegisterBits(SI4032_REG_MODULATION_MODE_C2, 0b00000011, type);
}

void Si4032_SetFrequencyDeviation(uint32_t frequency_hz) {
    // Because Si4032 in RS41 use 26 MHz clock
    // recalculate correct value
    uint16_t deviation_call = (frequency_hz*15+6)/13/SI4032_FREQUENCY_DEVIATION_STEP;
    // Set lower bits
    Si4032_Write(SI4032_REG_FREQUENCY_DEVIATION, (uint8_t)(deviation_call & 0xFF));
    // Set top MSB bit
    Si4032_AssignRegisterBits(SI4032_REG_MODULATION_MODE_C2,0b00000100, (uint8_t)(deviation_call >> 6));
}

void Si4032_SetFrequencyOffset(uint16_t offset_hz) {
    // Because Si4032 in RS41 use 26 MHz clock
    // recalculate correct value
    uint16_t offset = (offset_hz*15+6)/13/SI4032_FREQUENCY_OFFSET_STEP;
    // Set lower bits
    Si4032_Write(SI4032_REG_FREQUENCY_OFFSET_1, (uint8_t)(offset & 0xFF));
    // Set higher bits
    Si4032_Write(SI4032_REG_FREQUENCY_OFFSET_2, (uint8_t)((offset >> 8) & 0x03));
}

static uint8_t Si4032_SetRegisterBits(uint8_t reg, uint8_t mask) {
    // First read current register value
    uint8_t reg_current = Si4032_Read(reg);
    // Set bits to 1
    reg_current |= mask;
    // Write new register value
    Si4032_Write(reg,reg_current);
    return reg_current;
}

static uint8_t Si4032_ClearRegisterBits(uint8_t reg, uint8_t mask) {
    // First read current register value
    uint8_t reg_current = Si4032_Read(reg);
    // Set bits to 0
    reg_current &= ~(mask);
    // Write new register value
    Si4032_Write(reg,reg_current);
    return reg_current;
}

static uint8_t Si4032_AssignRegisterBits(uint8_t reg, uint8_t mask, uint8_t bits) {
    // First read current register value
    uint8_t reg_current = Si4032_Read(reg);
    // Assign bits
    reg_current = (reg_current & ~(mask)) | (mask & bits);
    // Write new register value
    Si4032_Write(reg,reg_current);
    return reg_current;
}

int8_t Si4032_GetTemperature(void) {
    // Set ADC input for temperature, set ADC reference
    Si4032_Write(SI4032_REG_ADC_CONFIG, 0x00);
    // Set temperature range, entsoffs = 1
    Si4032_Write(SI4032_REG_TEMPERATURE_SENSOR_CONTROL, 0x20);
    // Trigger ADC reading
    Si4032_Write(SI4032_REG_ADC_CONFIG, 0x80);
    // Read temperature value
    uint8_t temp = Si4032_Read(SI4032_REG_ADC_VALUE);
    int8_t temperatura = (int8_t)(-64 + (temp * 0.5));
    return temperatura;
}

uint16_t Si4032_GetBatteryVoltage(void) {
    // Return voltage in milivolts
    return (uint16_t)Si4032_Read(SI4032_REG_BATTERY_VOLTAGE_LEVEL)*50+1700;
}

void Si4032_PacketMode(float data_rate, uint32_t deviation, uint8_t packet_len) {
    Si4032_SetModulatioType(SI4032_TX_MODULATION_TYPE_GFSK);
    Si4032_SetModulatioSource(SI4032_TX_MODULATION_SOURCE_FIFO);
    Si4032_SetFrequencyOffset(0);
    Si4032_SetFrequencyDeviation(deviation); // In Hz
    Si4032_SetTxDataRate(data_rate); // in kbps
    // packet handling enable, CRC-16 CCITT on packtet data only, LSB first
    Si4032_Write(SI4032_REG_PACKET_DATA_CONROL,0x6C);
    // 4 header bytes, 4 SYNC bytes, fixed packet length
    Si4032_Write(SI4032_REG_PACKET_HEADER_CONTROL2,0x4E);
    // preamble: 85 nibble
    Si4032_Write(SI4032_REG_PACKET_PREAMBLE_LENGTH,0x55);
    // packets len, max 255 bytes
    Si4032_Write(SI4032_REG_PACKET_TRANS_PACKET_LEN,packet_len);
    // Vaisala RS41 header 8 bytes, use sync + header
    // little endian encoded, both bytewise and bitwise
    Si4032_Write(SI4032_REG_PACKET_SYNC_WORD3,0x10);
    Si4032_Write(SI4032_REG_PACKET_SYNC_WORD2,0xB6);
    Si4032_Write(SI4032_REG_PACKET_SYNC_WORD1,0xCA);
    Si4032_Write(SI4032_REG_PACKET_SYNC_WORD0,0x11);
    Si4032_Write(SI4032_REG_PACKET_TRANS_HEADER3,0x22);
    Si4032_Write(SI4032_REG_PACKET_TRANS_HEADER2,0x96);
    Si4032_Write(SI4032_REG_PACKET_TRANS_HEADER1,0x12);
    Si4032_Write(SI4032_REG_PACKET_TRANS_HEADER0,0xF8);
    // Clear TX fifo, Auto TX
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C2,0x81);
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C2,0x80);
}

void Si4032_WritePacket(const uint8_t *Data, uint8_t Len) {
    // fill the payload into the transmit FIFO
    for(uint8_t Idx=0; Idx<Len; Idx++){
        Si4032_Write(SI4032_REG_FIFO_ACCESS,Data[Idx]);
    }
    // Disable all other interrupts and enable the packet sent interrupt only.
    // This will be used for indicating the successful packet transmission for the MCU
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_1,0x04);
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_2,0x00);
    // Read interrupt status registers. It clear all pending interrupts and the nIRQ pin goes back to high.
    // nIRQ ?? Not connected on the Vaisala RS41 board?
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    _ItStatus2 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_2);
    // enable transmitter
    // The radio forms the packet and send it automatically.
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x49);
    // wait for the packet sent interrupt
    // The MCU just needs to wait for the 'ipksent' interrupt.
    // while(NIRQ == 1); ?? Not connected on the Vaisala RS41 board?
    // read interrupt status registers to release the interrupt flags
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    _ItStatus2 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_2);
}

void Si4032_Init(void) {
    // Set NSEL pin
    gpio_set(SI4032_NSEL_GPIO,SI4032_NSEL_PIN);
    // read interrupt status registers to clear the interrupt flags and release NIRQ pin
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    _ItStatus2 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_2);
    // Reset
    Si4032_SoftReset();
    // wait for chip ready interrupt from the radio (while the nIRQ pin is high)
    // while ( NIRQ == 1);  ?? Not connected on the Vaisala RS41 board?
    // read interrupt status registers to clear the interrupt flags and release NIRQ pin
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    _ItStatus2 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_2);
    // Init second part
    Si4032_Init2();
}

void Si4032_Init2(void) {
    // Set Frequency 433.125 MHz
    Si4032_SetFrequency(433.125f);
    // setting TX power 1.2 mW
    Si4032_SetTxPower(SI4032_TX_POWER_1_DBM);
    // Temperature Value Offset
    Si4032_Write(SI4032_REG_TEMPERATURE_VALUE_OFFSET, 0xF0);
    // Temperature Sensor Calibration
    Si4032_Write(SI4032_REG_TEMPERATURE_SENSOR_CONTROL, 0x00);
    // ADC configuration
    Si4032_Write(SI4032_REG_ADC_CONFIG, 0x80);
    // Zero frequency offset
    // The frequency offset can be calculated as
    // Offset = 156.25 Hz x (hbsel + 1) x fo[7:0]. fo[9:0] is a twos complement value.
    Si4032_SetFrequencyOffset(0);
    // Zero disables deviation between 0/1 bits
    // The frequency deviation can be calculated: Fd = 625 Hz x fd[8:0].
    Si4032_SetFrequencyDeviation(0);
    // initial unmodulated carrier, CW or RTTY modulation
    Si4032_SetModulatioType(SI4032_TX_MODULATION_TYPE_UNMOD);
    // Disable transmitter
    Si4032_DisableTx();
}

void Si4032_PrintRegisters(void) {
    int i;
    for(i=0x01;i<=0x18;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x1A;i<=0x1B;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x30;i<=0x31;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x33;i<=0x34;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x36;i<=0x3E;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x4F;i<=0x4F;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x62;i<=0x62;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x6D;i<=0x77;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x79;i<=0x7A;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x7C;i<=0x7D;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }
    for(i=0x7F;i<=0x7F;i++) {
        Si4032_Print(i,Si4032_Read(i));
    }

}

static void Si4032_Print(int reg, uint8_t value) {
    console_puts("R: 0x");
    if(reg < 16) {
        console_putc('0');
    }
    char buf[20];
    itoa(reg,buf,16);
    console_puts(buf);
    console_puts(" V: 0x");
    if(value < 16) {
        console_putc('0');
    }
    itoa(value,buf,16);
    console_puts(buf);
    console_puts("\n");
}
