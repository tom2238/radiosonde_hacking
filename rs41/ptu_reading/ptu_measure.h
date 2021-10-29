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

#ifndef PTU_MEASURE_H
#define PTU_MEASURE_H

// Vaisala RS41 PTU connection
// Vaisala RS41 external timer input
#define PTU_MEAS_OUT_TIMER TIM2
#define PTU_MEAS_OUT_TIMER_RCC RCC_TIM2
#define PTU_MEAS_OUT_TIMER_IRQ NVIC_TIM2_IRQ
#define PTU_MEAS_OUT_TIMER_RST RST_TIM2
#define PTU_MEAS_OUT_TIMER_PERIOD 0xFFFF

// TIM3 counter as stopwatch
#define PTU_FREQUENCY_TIMER TIM3
#define PTU_FREQUENCY_TIMER_RCC RCC_TIM3
#define PTU_FREQUENCY_TIMER_IRQ NVIC_TIM3_IRQ
#define PTU_FREQUENCY_TIMER_RST RST_TIM3
#define PTU_FREQUENCY_TIMER_PERIOD 0xFFFF
#define PTU_FREQUENCY_TIMER_CLOCK 24000000UL
#define PTU_FREQUENCY_TIMER_PULSE_LIMIT 1024

// Vaisala RS41 PTU pins
// Temperature
#define PTU_TEMP_REF1_GPIO GPIOB
#define PTU_TEMP_REF1_PIN GPIO6
#define PTU_TEMP_REF1_RCC RCC_GPIOA
#define PTU_TEMP_REF2_GPIO GPIOA
#define PTU_TEMP_REF2_PIN GPIO3
#define PTU_TEMP_REF2_RCC RCC_GPIOB
#define PTU_TEMP_HUMI_GPIO GPIOC
#define PTU_TEMP_HUMI_PIN GPIO14
#define PTU_TEMP_HUMI_RCC RCC_GPIOC
#define PTU_TEMP_SENSOR_GPIO GPIOC
#define PTU_TEMP_SENSOR_PIN GPIO15
#define PTU_TEMP_SENSOR_RCC RCC_GPIOC
#define PTU_TEMP_ACTIVATION_GPIO GPIOB
#define PTU_TEMP_ACTIVATION_PIN GPIO12
#define PTU_TEMP_ACTIVATION_RCC RCC_GPIOB
// Humidity
#define PTU_HUMI_REF1_GPIO GPIOB
#define PTU_HUMI_REF1_PIN GPIO4
#define PTU_HUMI_REF1_RCC RCC_GPIOB
#define PTU_HUMI_REF2_GPIO GPIOB
#define PTU_HUMI_REF2_PIN GPIO5
#define PTU_HUMI_REF2_RCC RCC_GPIOB
#define PTU_HUMI_SENSOR_GPIO GPIOB
#define PTU_HUMI_SENSOR_PIN GPIO3
#define PTU_HUMI_SENSOR_RCC RCC_GPIOB
#define PTU_HUMI_ACTIVATION_GPIO GPIOA
#define PTU_HUMI_ACTIVATION_PIN GPIO2
#define PTU_HUMI_ACTIVATION_RCC RCC_GPIOA

// PTU calculation constants
// PT1000 sensor
#define PTU_CONSTANT_PT1000_A0 -243.911
#define PTU_CONSTANT_PT1000_A1 0.187654
#define PTU_CONSTANT_PT1000_A2 8.2e-06
// Reference resistor values
#define PTU_REFERENCE_RESISTOR1_OHM 750.0f
#define PTU_REFERENCE_RESISTOR2_OHM 1100.0f

// PTU stopwatch timer/counter states
typedef enum {
    PTU_FREQ_STATE_IDLE = 1,
    PTU_FREQ_STATE_RUN,
    PTU_FREQ_STATE_DONE
}PTUFrequencyCounterState;

// PTU frequency information
typedef struct {
    uint32_t OverCapture;
    uint32_t Pulses;
    uint32_t PeriodTimer;
    uint32_t FrequencyHz;
    PTUFrequencyCounterState State;
}PTUFrequencyCounter;

// RAW PTU data
typedef struct {
    uint32_t temperature_ref1;
    uint32_t temperature_ref2;
    uint32_t temperature_sensor;
    uint32_t temperature_humi;
    uint32_t humidity_ref1;
    uint32_t humidity_ref2;
    uint32_t humidity_sensor;
}PTURAWData;

// Calculated PTU data from RAW values
typedef struct {
    float temperature_sensor;
    float temperature_humi;
    float humidity_sensor;
}PTUCalculatedData;

// Calibration data
typedef struct {
    float cal_T1[3];
    float cal_H[2];
    float cal_T2[3];
}PTUCalibrationData;

// Functions
// Public
void PTU_Init(void);
extern void PTU_EnableReferenceHeating(void);
extern void PTU_DisableReferenceHeating(void);
void PTU_MeasureTemperature(PTURAWData *rawdata);
void PTU_MeasureHumidity(PTURAWData *rawdata);
void PTU_CalculateData(PTURAWData *rawdata, PTUCalculatedData *caldata, PTUCalibrationData calibration);

#endif // PTU_MEASURE_H
