/*
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

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
#include "si4032.h"
#include "utils.h"

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

// Rate in bps
static uint32_t _rate_bps;

/**
 * @brief Si4032_Register Read or write operation with register
 * @param register_addr Register address
 * @param value Value write to register
 * @param write Write or read operation with register
 * @return Value read from register
 */
static inline uint8_t Si4032_Register(const uint8_t register_addr, uint8_t value, uint8_t write) {
    return Si4032_SpiTransaction(((write ? register_addr | SI4032_REGISTER_WRITE : register_addr) << 8) | value);
}

/**
 * @brief Si4032_SpiTransaction Write and read from SI4032 radio
 * @param data 16 bit SPI command for radio
 * @return Value read from register
 */
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

/**
 * @brief Si4032_Write Write value into radio register
 * @param reg Register address
 * @param value Value write to register
 * @return Value read from register
 */
static inline uint8_t Si4032_Write(uint8_t reg, uint8_t value) {
    return Si4032_SpiTransaction(((reg | SI4032_REGISTER_WRITE) << 8U) | value);
}

/**
 * @brief Si4032_Read Read value from radio register
 * @param reg Register address
 * @return Value read from register
 */
static inline uint8_t Si4032_Read(uint8_t reg) {
    return Si4032_SpiTransaction((reg << 8U) | 0xFFU);
}

/**
 * @brief Si4032_SetFrequency Set radio carrier frequency in range 208 MHz to 832 MHz
 * @param freq_in_mhz Carrier frequency in megahertz
 */
void Si4032_SetFrequency(const float freq_in_mhz) {
    uint8_t hbsel = (uint8_t) ((freq_in_mhz * (30.0f / SI4032_XTAL_CLOCK)) >= 480.0f ? 1 : 0);
    uint8_t fb = (uint8_t) ((((uint8_t)((freq_in_mhz * (30.0f / SI4032_XTAL_CLOCK)) / 10) - 24) - (24 * hbsel)) / (1 + hbsel));
    uint8_t gen_div  =  3;  // constant - not possible to change!
    uint16_t fc = (uint16_t) (((freq_in_mhz / ((SI4032_XTAL_CLOCK / gen_div) * (hbsel + 1))) - fb - 24) * 64000);
    Si4032_Write(SI4032_REG_FREQUENCY_BAND_SEL, (uint8_t) (0b01000000 | (fb & 0b11111) | ((hbsel & 0b1) << 5)));
    Si4032_Write(SI4032_REG_NOMINAL_CARRIER_1, (uint8_t) (((uint16_t)fc >> 8) & 0xff));
    Si4032_Write(SI4032_REG_NOMINAL_CARRIER_2, (uint8_t) ((uint16_t)fc & 0xff));
}

/**
 * @brief Si4032_GetVersion
 * @return Version number of SI4032 chip
 */
uint8_t Si4032_GetVersion(void) {
    return Si4032_Read(SI4032_REG_DEVICE_VERSION);
}

/**
 * @brief Si4032_DisableTx Disable radio transmission, PLL and XTAL is turn off
 */
void Si4032_DisableTx(void) {
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x40);
}

/**
 * @brief Si4032_SoftReset Reset all SI4032 registers to default state
 */
static void Si4032_SoftReset(void) {
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x80);
}

/**
 * @brief Si4032_EnableTx Enable radio transmission
 */
void Si4032_EnableTx(void) {
    // Modified to set the PLL and Crystal enable bits to high. Not sure if this makes much differents.
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x4B);
}

/**
 * @brief Si4032_InhibitTx Disable radio transmission, PLL and XTAL is turn on
 */
void Si4032_InhibitTx(void) {
    // Sleep mode, but with PLL idle mode enabled, in an attempt to reduce drift on key-up.
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x43);
}

/**
 * @brief Si4032_PacketTx Enable radio transmission for packet mode
 */
void Si4032_PacketTx(void) {
    /*enable transmitter*/
    //The radio forms the packet and send it automatically
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C1, 0x49);
}

/**
 * @brief Si4032_SetTxPower Set radio TX power from 1 to 20 dBm
 * @param power TX power value
 */
void Si4032_SetTxPower(uint8_t power) {
    Si4032_Write(SI4032_REG_TX_POWER, (power & 0x07));
}

/**
 * @brief Si4032_SetTxDataRate Set TX data rate in FIFO or PN9 mode
 * @param rate_bps Data rate in bits per second
 */
void Si4032_SetTxDataRate(uint32_t rate_bps) {
    // Check limits
    if(rate_bps > 256000) { // Rate is to hight
        rate_bps = 256000;
    }
    if(rate_bps < 123) { // Rate is to low
        rate_bps = 123;
    }
    // Save rate into private variable
    _rate_bps = rate_bps;
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

/**
 * @brief Si4032_SetModulatioSource Set radio data source
 * @param source GPIO, SDI, PN9 or FIFO source
 */
void Si4032_SetModulatioSource(uint8_t source) {
    Si4032_AssignRegisterBits(SI4032_REG_MODULATION_MODE_C2, 0b00110000, source);
}

/**
 * @brief Si4032_SetModulatioType Set signal modulation type
 * @param type UNMOD, OOK, FSK or GFSK modulation
 */
void Si4032_SetModulatioType(uint8_t type) {
    Si4032_AssignRegisterBits(SI4032_REG_MODULATION_MODE_C2, 0b00000011, type);
}

/**
 * @brief Si4032_SetFrequencyDeviation Set frequency deviation for (G)FSK mode
 * @param frequency_hz Deviation in hertz
 */
void Si4032_SetFrequencyDeviation(uint32_t frequency_hz) {
    // Because Si4032 in RS41 use 26 MHz clock
    // recalculate correct value
    uint16_t deviation_call = (frequency_hz*15+6)/13/SI4032_FREQUENCY_DEVIATION_STEP;
    // Set lower bits
    Si4032_Write(SI4032_REG_FREQUENCY_DEVIATION, (uint8_t)(deviation_call & 0xFF));
    // Set top MSB bit
    Si4032_AssignRegisterBits(SI4032_REG_MODULATION_MODE_C2,0b00000100, (uint8_t)(deviation_call >> 6));
}

/**
 * @brief Si4032_SetFrequencyOffset Add frequency offset to the carrier frequency
 * @param offset_hz Offset frequency in hertz
 */
void Si4032_SetFrequencyOffset(uint16_t offset_hz) {
    // Because Si4032 in RS41 use 26 MHz clock
    // recalculate correct value
    uint16_t offset = (offset_hz*15+6)/13/SI4032_FREQUENCY_OFFSET_STEP;
    // Set lower bits
    Si4032_Write(SI4032_REG_FREQUENCY_OFFSET_1, (uint8_t)(offset & 0xFF));
    // Set higher bits
    Si4032_Write(SI4032_REG_FREQUENCY_OFFSET_2, (uint8_t)((offset >> 8) & 0x03));
}

/**
 * @brief Si4032_SetRegisterBits Set selected bit to 1 in register
 * @param reg Register address
 * @param mask Binary mask, 1 set bit to 1, 0 no change
 * @return New register value
 */
static uint8_t Si4032_SetRegisterBits(uint8_t reg, uint8_t mask) {
    // First read current register value
    uint8_t reg_current = Si4032_Read(reg);
    // Set bits to 1
    reg_current |= mask;
    // Write new register value
    Si4032_Write(reg,reg_current);
    return reg_current;
}

/**
 * @brief Si4032_ClearRegisterBits Set selected bit to 0 in register
 * @param reg Register address
 * @param mask Binary mask, 1 set bit to 0, 0 no change
 * @return New register value
 */
static uint8_t Si4032_ClearRegisterBits(uint8_t reg, uint8_t mask) {
    // First read current register value
    uint8_t reg_current = Si4032_Read(reg);
    // Set bits to 0
    reg_current &= ~(mask);
    // Write new register value
    Si4032_Write(reg,reg_current);
    return reg_current;
}

/**
 * @brief Si4032_AssignRegisterBits Set selected bits to 1 or 0
 * @param reg Register address
 * @param mask Binary mask, set to 1 to change bit, 0 no change
 * @param bits New bit value
 * @return New register value
 */
static uint8_t Si4032_AssignRegisterBits(uint8_t reg, uint8_t mask, uint8_t bits) {
    // First read current register value
    uint8_t reg_current = Si4032_Read(reg);
    // Assign bits
    reg_current = (reg_current & ~(mask)) | (mask & bits);
    // Write new register value
    Si4032_Write(reg,reg_current);
    return reg_current;
}

/**
 * @brief Si4032_GetTemperature Return SI4032 radio temperature in celsius degree
 * @return Return SI4032 radio temperature in celsius degree
 */
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

/**
 * @brief Si4032_GPIOSet
 * @param gpio
 */
void Si4032_GPIOSet(uint8_t gpio_drv) {
    // Driving Capability Setting = 0x0
    // Pullup Resistor Enable = 0
    // Pin Function Select = VDD
    switch (gpio_drv) {
    case SI4032_GPIO_PORT_0:
        Si4032_Write(SI4032_REG_GPIO0_CONFIG,0b00011101);
        break;
    case SI4032_GPIO_PORT_1:
        Si4032_Write(SI4032_REG_GPIO1_CONFIG,0b00011101);
        break;
    case SI4032_GPIO_PORT_2:
        Si4032_Write(SI4032_REG_GPIO2_CONFIG,0b00011101);
        break;
    default:
        break;
    }
}

/**
 * @brief Si4032_GPIOClear
 * @param gpio_drv
 */
void Si4032_GPIOClear(uint8_t gpio_drv) {
    // Driving Capability Setting = 0x0
    // Pullup Resistor Enable = 0
    // Pin Function Select = GND
    switch (gpio_drv) {
    case SI4032_GPIO_PORT_0:
        Si4032_Write(SI4032_REG_GPIO0_CONFIG,0b00011111);
        break;
    case SI4032_GPIO_PORT_1:
        Si4032_Write(SI4032_REG_GPIO1_CONFIG,0b00011111);
        break;
    case SI4032_GPIO_PORT_2:
        Si4032_Write(SI4032_REG_GPIO2_CONFIG,0b00011111);
        break;
    default:
        break;
    }
}

/**
 * @brief Si4032_GetBatteryVoltage Return SI4032 radio voltage in milivolts
 * @return Return SI4032 radio voltage in milivolts
 */
uint16_t Si4032_GetBatteryVoltage(void) {
    // Return voltage in milivolts
    return (uint16_t)Si4032_Read(SI4032_REG_BATTERY_VOLTAGE_LEVEL)*50+1700;
}

/**
 * @brief Si4032_PacketMode Set radio for packet mode and FIFO source
 * @param packet_type Packet type, short (<=64), long (64<=255) or infinitive (255 < inf) bytes
 * @param data_rate Packet data rate in bits per second
 * @param deviation Frequency deviation for (G)FSK mode in hertz
 * @param packet_len Packet length in short or long mode. Max length is 255 bytes
 * @param preamble_length Packet preamble length. Max is 255 nibbles
 */
void Si4032_PacketMode(enum SI4032_PACKET_TYPE packet_type, uint32_t data_rate, uint32_t deviation, uint8_t packet_len, uint8_t preamble_length) {
    // Common options
    Si4032_SetModulatioType(SI4032_TX_MODULATION_TYPE_GFSK);
    Si4032_SetModulatioSource(SI4032_TX_MODULATION_SOURCE_FIFO);
    Si4032_SetFrequencyOffset(0); // Zero frequency offset
    Si4032_SetFrequencyDeviation(deviation); // In Hz
    Si4032_SetTxDataRate(data_rate); // in kbps
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
    // packets len, max 255 bytes
    Si4032_Write(SI4032_REG_PACKET_TRANS_PACKET_LEN,packet_len);
    // preamble, max 255 nibbles
    Si4032_Write(SI4032_REG_PACKET_PREAMBLE_LENGTH,preamble_length);
    // Packet settings
    switch (packet_type) {
    case PACKET_TYPE_INFINITE:
        // packet handling disable, no CRC-16, LSB first
        Si4032_Write(SI4032_REG_PACKET_DATA_CONROL,0x40);
        // No header bytes, 1 SYNC bytes, fixed packet length
        Si4032_Write(SI4032_REG_PACKET_HEADER_CONTROL2,0x08);
        break;
    case PACKET_TYPE_LONG:
    case PACKET_TYPE_SHORT:
    default:
        // packet handling enable, no CRC-16, LSB first
        Si4032_Write(SI4032_REG_PACKET_DATA_CONROL,0x48);
        // 4 header bytes, 4 SYNC bytes, fixed packet length
        Si4032_Write(SI4032_REG_PACKET_HEADER_CONTROL2,0x4E);
        break;
    }
    // FIFO Full Threshold
    Si4032_Write(SI4032_REG_TX_FIFO_C1,SI4032_FIFO_FULL_THRESHOLD); // 55 bytes
    // FIFO Empty Threshold
    Si4032_Write(SI4032_REG_TX_FIFO_C2,SI4032_FIFO_EMPTY_THRESHOLD); // 8 bytes
    // Clear TX fifo, Auto TX
    Si4032_ClearFIFO();
}

/**
 * @brief Si4032_WriteShortPacket Write packet data, max 64 bytes
 * @param Data Packet data array
 * @param Len Packet length, must be max 64 bytes long
 */
void Si4032_WriteShortPacket(const uint8_t *Data, uint8_t Len) {
    // Preamble and header is added in Si4032
    // Clear FIFO content on start
    Si4032_ClearFIFO();
    // fill the payload into the transmit FIFO
    for(uint8_t Idx=0; Idx<Len; Idx++){
        Si4032_Write(SI4032_REG_FIFO_ACCESS,Data[Idx]);
    }
    // enable transmitter
    // The radio forms the packet and send it automatically.
    Si4032_PacketTx();
    // wait for the packet sent interrupt
    // The MCU just needs to wait for the 'ipksent' interrupt.
    // Wait for send
    while(!Si4032_IsPacketSent());
    // Disable transmitter
    Si4032_DisableTx();
}

/**
 * @brief Si4032_WritePacketData Fill the payload into the transmit FIFO
 * @param *Data uint8_t Packet payload array
 * @param start uint16_t Start position in array
 * @param size uint16_t Size of data loaded into FIFO from array
 */
void Si4032_WritePacketData(const uint8_t *Data, uint16_t start, uint16_t size) {
    // fill the payload into the transmit FIFO
    for(uint16_t Idx=start; Idx<start+size; Idx++){
        Si4032_Write(SI4032_REG_FIFO_ACCESS,Data[Idx]);
    }
}

/**
 * @brief Si4032_IsFIFOEmpty Check if TX FIFO is empty
 * @return If is empty, return 1
 */
uint8_t Si4032_IsFIFOEmpty(void) {
    volatile unsigned int i;
    for(i=0;i<SI4032_INTERRUPT_CHECK_DELAY_CONSTANT/_rate_bps;i++) {}
    // Return 1 if FIFO is empty
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    return (_ItStatus1 >> 5) & 0x01;
}

/**
 * @brief Si4032_IsPacketSent Check if all packet payload was sended
 * @return If was sended, return 1
 */
uint8_t Si4032_IsPacketSent(void) {
    volatile unsigned int i;
    for(i=0;i<SI4032_INTERRUPT_CHECK_DELAY_CONSTANT/_rate_bps;i++) {}
    // Return 1 if packet was sended
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    return (_ItStatus1 >> 2) & 0x01;
}

/**
 * @brief Si4032_EnablePacketSentInterrupt Disable all other interrupts and enable the packet sent interrupt only, for nIRQ pin
 */
void Si4032_EnablePacketSentInterrupt(void) {
    //Disable all other interrupts and enable the packet sent interrupt only.
    //This will be used for indicating the successfull packet transmission for the MCU
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_1, 0x04);
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_2, 0x00);
    Si4032_ClearInterruptStatus();
}

/**
 * @brief Si4032_EnableFIFOEmptyInterrupt Disable all other interrupts and enable the FIFO almost empty interrupt only, for nIRQ pin
 */
void Si4032_EnableFIFOEmptyInterrupt(void) {
    //Disable all other interrupts and enable the FIFO almost empty interrupt only.
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_1, 0x20);
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_2, 0x00);
    Si4032_ClearInterruptStatus();
}

/**
 * @brief Si4032_ClearInterruptStatus Read radio interrupt registers and save it into private variables
 */
void Si4032_ClearInterruptStatus(void) {
    //Read interrupt status regsiters. It clear all pending interrupts and the nIRQ pin goes back to high.
    // nIRQ ?? Not connected on the Vaisala RS41 board?
    _ItStatus1 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_1);
    _ItStatus2 = Si4032_Read(SI4032_REG_INTERRUPT_STATUS_2);
}

/**
 * @brief Si4032_ClearFIFO Clear TX FIFO content
 */
void Si4032_ClearFIFO(void) {
    // Clear TX fifo, Auto TX
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C2,0x81);
    Si4032_Write(SI4032_REG_OPERATING_FUNCTION_C2,0x80);
}

/**
 * @brief Si4032_Init Initialize SI4032 radio, first part
 */
void Si4032_Init(void) {
    // Set NSEL pin
    gpio_set(SI4032_NSEL_GPIO,SI4032_NSEL_PIN);
    // read interrupt status registers to clear the interrupt flags and release NIRQ pin
    Si4032_ClearInterruptStatus();
    // Reset
    Si4032_SoftReset();
    // Set interrupt masks to zero
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_1,0x00);
    Si4032_Write(SI4032_REG_INTERRUPT_ENABLE_2,0x00);
    // wait for chip ready interrupt from the radio (while the nIRQ pin is high)
    // while ( NIRQ == 1);  ?? Not connected on the Vaisala RS41 board?
    // read interrupt status registers to clear the interrupt flags and release NIRQ pin
    Si4032_ClearInterruptStatus();
    // Init second part
    Si4032_Init2();
}

/**
 * @brief Si4032_Init2 Initialize SI4032 radio, second part
 */
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

/**
 * @brief Si4032_PrintRegisters Print all usefull SI4032 registers on UART
 */
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

/**
 * @brief Si4032_Print Print register address and value on UART in HEX
 * @param reg Register address to print
 * @param value Register value to print
 */
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
