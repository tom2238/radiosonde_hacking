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
static void Si4032_Print(int reg, uint8_t value);

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

void Si4032_SoftReset(void) {
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

void Si4032_Init(void) {
    // Set NSEL pin
    gpio_set(SI4032_NSEL_GPIO,SI4032_NSEL_PIN);
    // Reset
    Si4032_SoftReset();
    // Init second part
    Si4032_Init2();
}

void Si4032_Init2(void) {
    // setting TX power 1.2 mW
    Si4032_Write(SI4032_REG_TX_POWER, (SI4032_TX_POWER_1_DBM & 0x07));
    // Temperature Value Offset
    Si4032_Write(SI4032_REG_TEMPERATURE_VALUE_OFFSET, 0xF0);
    // Temperature Sensor Calibration
    Si4032_Write(SI4032_REG_TEMPERATURE_SENSOR_CONTROL, 0x00);
    // ADC configuration
    Si4032_Write(SI4032_REG_ADC_CONFIG, 0x80);
    // Zero frequency offset
    Si4032_Write(SI4032_REG_FREQUENCY_OFFSET_1, 0x00);
    Si4032_Write(SI4032_REG_FREQUENCY_OFFSET_2, 0x00);
    // The frequency deviation can be calculated: Fd = 625 Hz x fd[8:0].
    // Zero disables deviation between 0/1 bits
    Si4032_Write(SI4032_REG_FREQUENCY_DEVIATION, 0);
    // initial unmodulated carrier, CW or RTTY modulation
    Si4032_Write(SI4032_REG_MODULATION_MODE_C2, 0x00);
    // Set Frequency 433.125 MHz
    Si4032_SetFrequency(433.125f);
    // Enable transmitter
    Si4032_EnableTx();
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
