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

// Private
static uint8_t Si4032_Register(const uint8_t register_addr, uint8_t value, uint8_t write);
static uint8_t Si4032_SpiTransaction(uint16_t data);

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

void Si4032_SetFrequency(const float freq_in_mhz) {
    uint8_t hbsel = (uint8_t) ((freq_in_mhz * (30.0f / SI4032_XTAL_CLOCK)) >= 480.0f ? 1 : 0);
    uint8_t fb = (uint8_t) ((((uint8_t)((freq_in_mhz * (30.0f / SI4032_XTAL_CLOCK)) / 10) - 24) - (24 * hbsel)) / (1 + hbsel));
    uint8_t gen_div  =  3;  // constant - not possible to change!
    uint16_t fc = (uint16_t) (((freq_in_mhz / ((SI4032_XTAL_CLOCK / gen_div) * (hbsel + 1))) - fb - 24) * 64000);
    Si4032_Register(SI4032_REG_FREQUENCY_BAND_SEL, (uint8_t) (0b01000000 | (fb & 0b11111) | ((hbsel & 0b1) << 5)), SI4032_WRITE);
    Si4032_Register(SI4032_REG_NOMINAL_CARRIER_1, (uint8_t) (((uint16_t)fc >> 8) & 0xff), SI4032_WRITE);
    Si4032_Register(SI4032_REG_NOMINAL_CARRIER_2, (uint8_t) ((uint16_t)fc & 0xff), SI4032_WRITE);
}

uint8_t Si4032_GetVersion(void) {
    return Si4032_Register(SI4032_REG_DEVICE_VERSION,0xFF,SI4032_READ);
}

void Si4032_DisableTx(void) {
    Si4032_Register(SI4032_REG_OPERATING_FUNCTION_C1, 0x40, SI4032_WRITE);
}

void Si4032_SoftReset(void) {
    Si4032_Register(SI4032_REG_OPERATING_FUNCTION_C1, 0x80, SI4032_WRITE);
}

void Si4032_EnableTx(void) {
    // Modified to set the PLL and Crystal enable bits to high. Not sure if this makes much differents.
    Si4032_Register(SI4032_REG_OPERATING_FUNCTION_C1, 0x4B, SI4032_WRITE);
}

void Si4032_InhibitTx(void) {
    // Sleep mode, but with PLL idle mode enabled, in an attempt to reduce drift on key-up.
    Si4032_Register(SI4032_REG_OPERATING_FUNCTION_C1, 0x43, 1);
}

int8_t Si4032_GetTemperature(void) {
    // Set ADC input for temperature, set ADC reference
    Si4032_Register(SI4032_REG_ADC_CONFIG, 0x00, SI4032_WRITE);
    // Set temperature range, entsoffs = 1
    Si4032_Register(SI4032_REG_TEMPERATURE_SENSOR_CONTROL, 0x20, SI4032_WRITE);
    // Trigger ADC reading
    Si4032_Register(SI4032_REG_ADC_CONFIG, 0x80, SI4032_WRITE);
    // Read temperature value
    uint8_t temp = Si4032_Register(SI4032_REG_ADC_VALUE, 0xFF, SI4032_READ);
    int8_t temperatura = (int8_t)(-64 + (temp * 0.5));
    return temperatura;
}

uint16_t Si4032_GetBatteryVoltage(void) {
    // Return voltage in milivolts
    return (uint16_t)Si4032_Register(SI4032_REG_BATTERY_VOLTAGE_LEVEL, 0xFF, SI4032_READ)*50+1700;
}

void Si4032_Init(void) {
    // Set NSEL pin
    gpio_set(SI4032_NSEL_GPIO,SI4032_NSEL_PIN);
    // Reset
    Si4032_SoftReset();
    // Set Frequency
    Si4032_SetFrequency(433.0f);
    // Set PLL offset to 0
    Si4032_Register(SI4032_REG_FREQUENCY_OFFSET_1, 0x00, 1);
    // setting TX power 1 mW
    Si4032_Register(SI4032_REG_TX_POWER, 00 | (0 & 0x0007), 1);
    // initial RTTY modulation
    Si4032_Register(SI4032_REG_MODULATION_MODE_C2, 0x00, 1);
    // Temperature Value Offset
    Si4032_Register(SI4032_REG_TEMPERATURE_VALUE_OFFSET, 0xF0, 1);
    // Temperature Sensor Calibration
    Si4032_Register(SI4032_REG_TEMPERATURE_SENSOR_CONTROL, 0x00, 1);
    // ADC configuration
    Si4032_Register(SI4032_REG_ADC_CONFIG, 0x80, 1);
    // Disable TX
    Si4032_DisableTx();
    Si4032_Register(SI4032_REG_MODULATION_MODE_C2, 0x00, 1);
    Si4032_Register(SI4032_REG_FREQUENCY_BAND_SEL, 0x20, SI4032_WRITE);
    Si4032_Register(SI4032_REG_NOMINAL_CARRIER_1, 0xF8, SI4032_WRITE);
    Si4032_Register(SI4032_REG_NOMINAL_CARRIER_2, 0x14, SI4032_WRITE);
}

void Si4032_Init2(void) {
    // Set Frequency
    //Si4032_SetFrequency(433.0f);
    // initial RTTY modulation
    Si4032_Register(SI4032_REG_MODULATION_MODE_C2, 0x00, 1);
    Si4032_Register(SI4032_REG_FREQUENCY_BAND_SEL, 0x20, SI4032_WRITE);
    Si4032_Register(SI4032_REG_NOMINAL_CARRIER_1, 0xF8, SI4032_WRITE);
    Si4032_Register(SI4032_REG_NOMINAL_CARRIER_2, 0x14, SI4032_WRITE);
}
