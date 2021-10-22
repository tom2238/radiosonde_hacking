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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/timer.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/timer.h>
// Another libraries
#include <stdint.h>
#include "ptu_measure.h"
#include "si4032.h"
#include "utils.h"

// Private
static inline void PTU_MeasureCounterInit(void);
static inline void PTU_FrequencyTimerInit(void);
static void PTU_FrequencyMeasure(PTUFrequencyCounter *fcounter);
static inline void PTU_EnableTemperatureMeasure(void);
static inline void PTU_DisableTemperatureMeasure(void);
static inline void PTU_EnableHumidityMeasure(void);
static inline void PTU_DisableHumidityMeasure(void);
static inline void PTU_TemperatureSelectREF1(void);
static inline void PTU_TemperatureDeselectREF1(void);
static inline void PTU_TemperatureSelectHumidity(void);
static inline void PTU_TemperatureDeselectHumidity(void);
static inline void PTU_TemperatureSelectSensor(void);
static inline void PTU_TemperatureDeselectSensor(void);
static inline void PTU_TemperatureSelectREF2(void);
static inline void PTU_TemperatureDeselectREF2(void);
static inline void PTU_HumiditySelectREF1(void);
static inline void PTU_HumidityDeselectREF1(void);
static inline void PTU_HumiditySelectSensor(void);
static inline void PTU_HumidityDeselectSensor(void);
static inline void PTU_HumiditySelectREF2(void);
static inline void PTU_HumidityDeselectREF2(void);

// Frequency information
static PTUFrequencyCounter _ptu_frequency;

/**
 * @brief PTU_Init
 */
void PTU_Init(void) {
    // Enable PTU pins clock
    // Temperature
    rcc_periph_clock_enable(PTU_TEMP_REF1_RCC);
    rcc_periph_clock_enable(PTU_TEMP_REF2_RCC);
    rcc_periph_clock_enable(PTU_TEMP_HUMI_RCC);
    rcc_periph_clock_enable(PTU_TEMP_SENSOR_RCC);
    rcc_periph_clock_enable(PTU_TEMP_ACTIVATION_RCC);
    // Humidity
    rcc_periph_clock_enable(PTU_HUMI_REF1_RCC);
    rcc_periph_clock_enable(PTU_HUMI_REF2_RCC);
    rcc_periph_clock_enable(PTU_HUMI_SENSOR_RCC);
    rcc_periph_clock_enable(PTU_HUMI_ACTIVATION_RCC);

    // Set PTU direction
    // Temperature
    gpio_set_mode(PTU_TEMP_REF1_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_REF1_PIN);
    gpio_set_mode(PTU_TEMP_REF2_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_REF2_PIN);
    gpio_set_mode(PTU_TEMP_HUMI_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_HUMI_PIN);
    gpio_set_mode(PTU_TEMP_SENSOR_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_SENSOR_PIN);
    gpio_set_mode(PTU_TEMP_ACTIVATION_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_TEMP_ACTIVATION_PIN);
    // Humidity
    gpio_set_mode(PTU_HUMI_REF1_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_REF1_PIN);
    gpio_set_mode(PTU_HUMI_REF2_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_REF2_PIN);
    gpio_set_mode(PTU_HUMI_SENSOR_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_SENSOR_PIN);
    gpio_set_mode(PTU_HUMI_ACTIVATION_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PTU_HUMI_ACTIVATION_PIN);

    // Reset PTU pins
    PTU_EnableReferenceHeating();
    PTU_EnableTemperatureMeasure();
    PTU_TemperatureSelectREF1();
    PTU_TemperatureSelectREF2();
    PTU_TemperatureSelectHumidity();
    PTU_TemperatureSelectSensor();
    PTU_EnableHumidityMeasure();
    PTU_HumiditySelectREF1();
    PTU_HumiditySelectREF2();
    PTU_HumiditySelectSensor();
    delay(10);
    PTU_TemperatureDeselectREF1();
    PTU_TemperatureDeselectREF2();
    PTU_TemperatureDeselectHumidity();
    PTU_TemperatureDeselectSensor();
    PTU_DisableTemperatureMeasure();
    PTU_HumidityDeselectREF1();
    PTU_HumidityDeselectREF2();
    PTU_HumidityDeselectSensor();
    PTU_DisableHumidityMeasure();
    PTU_DisableReferenceHeating();

    // Init PA1 as input capture counter
    PTU_MeasureCounterInit();
    // Init TIM3 timer as stopwatch for time and frequency calculation
    PTU_FrequencyTimerInit();
    // Reset frequency info
    _ptu_frequency.FrequencyHz = 0;
    _ptu_frequency.OverCapture = 0;
    _ptu_frequency.PeriodTimer = 0;
    _ptu_frequency.Pulses = 0;
    _ptu_frequency.State = PTU_FREQ_STATE_IDLE;
}

/**
 * @brief PTU_MeasureCounterInit
 */
static inline void PTU_MeasureCounterInit(void) {
    // Input capture timer setup
    // Enable clock
    rcc_periph_clock_enable(PTU_MEAS_OUT_TIMER_RCC);
    // Reset peripheral
    rcc_periph_reset_pulse(PTU_MEAS_OUT_TIMER_RST);
    // Clock prescaler is 0, maximum accuracy
    timer_set_prescaler(PTU_MEAS_OUT_TIMER,0);
    // Counting up
    timer_direction_up(PTU_MEAS_OUT_TIMER);
    // Typically, set maximum period
    timer_set_period(PTU_MEAS_OUT_TIMER,PTU_MEAS_OUT_TIMER_PERIOD);
    // No clock divider
    timer_set_clock_division(PTU_MEAS_OUT_TIMER,TIM_CR1_CKD_CK_INT);
    // Enable Auto-Reload Buffering.
    timer_enable_preload(PTU_MEAS_OUT_TIMER);
    // Reset master
    timer_set_master_mode(PTU_MEAS_OUT_TIMER,TIM_CR2_MMS_RESET);
    // Disable slave mode
    timer_slave_set_mode(PTU_MEAS_OUT_TIMER,TIM_SMCR_SMS_OFF);
    // Rising edge
    timer_ic_set_polarity(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_RISING);
    // Input on PA1 pin, channel 2
    timer_ic_set_input(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_IN_TI2);
    // IC prescaler is off
    timer_ic_set_prescaler(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_PSC_OFF);
    // IC lowpass filter is TimerClock/(8*8)
    timer_ic_set_filter(PTU_MEAS_OUT_TIMER,TIM_IC2,TIM_IC_DTF_DIV_8_N_8);
    // Enable input capture
    timer_ic_enable(PTU_MEAS_OUT_TIMER,TIM_IC2);
    // Enable IRQ from timer
    nvic_enable_irq(PTU_MEAS_OUT_TIMER_IRQ);
    // Enable timer counter
    timer_enable_counter(PTU_MEAS_OUT_TIMER);
    // Enable channel 2 compare interrupt
    timer_enable_irq(PTU_MEAS_OUT_TIMER, TIM_DIER_CC2IE);
}

/**
 * @brief PTU_FrequencyTimerInit
 */
static inline void PTU_FrequencyTimerInit(void) {
    // Frequency timer setup
    // Enable clock
    rcc_periph_clock_enable(PTU_FREQUENCY_TIMER_RCC);
    // Reset peripheral
    rcc_periph_reset_pulse(PTU_FREQUENCY_TIMER_RST);
    // No clock divider, edge alignment, counting up
    timer_set_mode(PTU_FREQUENCY_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // Clock prescaler is 0, maximum accuracy
    timer_set_prescaler(PTU_FREQUENCY_TIMER,0);
    // Disable Auto-Reload Buffering.
    timer_disable_preload(PTU_FREQUENCY_TIMER);
    // Enable the Timer to Run Continuously.
    timer_continuous_mode(PTU_FREQUENCY_TIMER);
    // Typically, set maximum period
    timer_set_period(PTU_FREQUENCY_TIMER, PTU_FREQUENCY_TIMER_PERIOD);
    // Enable IRQ from timer
    nvic_enable_irq(PTU_FREQUENCY_TIMER_IRQ);
    // Enable timer overflow interrupt
    timer_enable_irq(PTU_FREQUENCY_TIMER, TIM_DIER_UIE);
}

/**
 * @brief tim2_isr
 */
void tim2_isr(void) {
    if (timer_get_flag(PTU_MEAS_OUT_TIMER, TIM_SR_CC2IF)) {
        if (_ptu_frequency.State == PTU_FREQ_STATE_RUN) {
            _ptu_frequency.Pulses++;
            if(_ptu_frequency.Pulses > PTU_FREQUENCY_TIMER_PULSE_LIMIT) {
                _ptu_frequency.PeriodTimer = timer_get_counter(PTU_FREQUENCY_TIMER);
                _ptu_frequency.State = PTU_FREQ_STATE_DONE;
                timer_disable_counter(PTU_FREQUENCY_TIMER);
            }
        }
        timer_clear_flag(PTU_MEAS_OUT_TIMER, TIM_SR_CC2IF);
    }
}

/**
 * @brief tim3_isr
 */
void tim3_isr(void) {
    if(timer_get_flag(PTU_FREQUENCY_TIMER, TIM_SR_UIF)) {
        _ptu_frequency.OverCapture += 65536;
        timer_clear_flag(PTU_FREQUENCY_TIMER, TIM_SR_UIF);
    }
}

/**
 * @brief PTU_EnableReferenceHeating
 */
inline void PTU_EnableReferenceHeating(void) {
    Si4032_GPIOSet(SI4032_GPIO_PORT_1);
}

/**
 * @brief PTU_DisableReferenceHeating
 */
inline void PTU_DisableReferenceHeating(void) {
    Si4032_GPIOClear(SI4032_GPIO_PORT_1);
}

/**
 * @brief PTU_FrequencyMeasure
 * @param fcounter
 */
static void PTU_FrequencyMeasure(PTUFrequencyCounter *fcounter) {
    // Timeout in ms
    uint16_t timeout = 100;
    // Reset before measure
    fcounter->State = PTU_FREQ_STATE_RUN;
    fcounter->OverCapture = 0;
    fcounter->Pulses = 0;
    fcounter->PeriodTimer = 0;
    fcounter->FrequencyHz = 0;
    // Reset the counter. This will generate one extra overflow for next measurement.
    // In case of nothing got counted, manually generate a reset to keep consistency.
    timer_set_counter(PTU_FREQUENCY_TIMER,0);
    // Enable the counter
    timer_enable_counter(PTU_FREQUENCY_TIMER);
    // Wait for ticks or timeout
    while(fcounter->Pulses <= PTU_FREQUENCY_TIMER_PULSE_LIMIT && timeout-- > 0) {
        delay(1);
    }
    // Timeout, low frequency or short timeout
    // Because if timeout reached, add one into timeout variable to get zero.
    timeout++;
    if(!timeout) {
        fcounter->FrequencyHz = 0;
        fcounter->PeriodTimer = 0;
    } else {
        // Frequency is correct
        fcounter->PeriodTimer += fcounter->OverCapture;
        fcounter->FrequencyHz = (PTU_FREQUENCY_TIMER_CLOCK)/(fcounter->PeriodTimer/PTU_FREQUENCY_TIMER_PULSE_LIMIT);
    }
}

/**
 * @brief PTU_EnableTemperatureMeasure
 */
static inline void PTU_EnableTemperatureMeasure(void) {
    gpio_set(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
}

/**
 * @brief PTU_DisableTemperatureMeasure
 */
static inline void PTU_DisableTemperatureMeasure(void) {
    gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
}

/**
 * @brief PTU_EnableHumidityMeasure
 */
static inline void PTU_EnableHumidityMeasure(void) {
    gpio_set(PTU_HUMI_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
}

/**
 * @brief PTU_DisableHumidityMeasure
 */
static inline void PTU_DisableHumidityMeasure(void) {
    gpio_clear(PTU_HUMI_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
}

/**
 * @brief PTU_TemperatureSelectREF1
 */
static inline void PTU_TemperatureSelectREF1(void) {
    gpio_set(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
}

/**
 * @brief PTU_TemperatureDeselectREF1
 */
static inline void PTU_TemperatureDeselectREF1(void) {
    gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
}

/**
 * @brief PTU_TemperatureSelectHumidity
 */
static inline void PTU_TemperatureSelectHumidity(void) {
    gpio_set(PTU_TEMP_HUMI_GPIO,PTU_TEMP_HUMI_PIN);
}

/**
 * @brief PTU_TemperatureDeselectHumidity
 */
static inline void PTU_TemperatureDeselectHumidity(void) {
    gpio_clear(PTU_TEMP_HUMI_GPIO,PTU_TEMP_HUMI_PIN);
}

/**
 * @brief PTU_TemperatureSelectSensor
 */
static inline void PTU_TemperatureSelectSensor(void) {
    gpio_set(PTU_TEMP_SENSOR_GPIO,PTU_TEMP_SENSOR_PIN);
}

/**
 * @brief PTU_TemperatureDeselectSensor
 */
static inline void PTU_TemperatureDeselectSensor(void) {
    gpio_clear(PTU_TEMP_SENSOR_GPIO,PTU_TEMP_SENSOR_PIN);
}

/**
 * @brief PTU_TemperatureSelectREF2
 */
static inline void PTU_TemperatureSelectREF2(void) {
    gpio_set(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
}

/**
 * @brief PTU_TemperatureDeselectREF2
 */
static inline void PTU_TemperatureDeselectREF2(void) {
    gpio_clear(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
}

/**
 * @brief PTU_HumiditySelectREF1
 */
static inline void PTU_HumiditySelectREF1(void) {
    gpio_set(PTU_HUMI_REF1_GPIO,PTU_HUMI_REF1_PIN);
}

/**
 * @brief PTU_HumidityDeselectREF1
 */
static inline void PTU_HumidityDeselectREF1(void) {
    gpio_clear(PTU_HUMI_REF1_GPIO,PTU_HUMI_REF1_PIN);
}

/**
 * @brief PTU_HumiditySelectSensor
 */
static inline void PTU_HumiditySelectSensor(void) {
    gpio_set(PTU_HUMI_SENSOR_GPIO,PTU_HUMI_SENSOR_PIN);
}

/**
 * @brief PTU_HumidityDeselectSensor
 */
static inline void PTU_HumidityDeselectSensor(void) {
    gpio_clear(PTU_HUMI_SENSOR_GPIO,PTU_HUMI_SENSOR_PIN);
}

/**
 * @brief PTU_HumiditySelectREF2
 */
static inline void PTU_HumiditySelectREF2(void) {
    gpio_set(PTU_HUMI_REF2_GPIO,PTU_HUMI_REF2_PIN);
}

/**
 * @brief PTU_HumidityDeselectREF2
 */
static inline void PTU_HumidityDeselectREF2(void) {
    gpio_clear(PTU_HUMI_REF2_GPIO,PTU_HUMI_REF2_PIN);
}

/**
 * @brief PTU_MeasureTemperature
 * @param rawdata
 */
void PTU_MeasureTemperature(PTURAWData *rawdata) {
    // REF1 750 ohm resistor
    PTU_EnableTemperatureMeasure();
    PTU_TemperatureSelectREF1();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_TemperatureDeselectREF1();
    PTU_DisableTemperatureMeasure();
    rawdata->temperature_ref1 = _ptu_frequency.FrequencyHz;
    // Humidity temperature
    PTU_EnableTemperatureMeasure();
    PTU_TemperatureSelectHumidity();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_TemperatureDeselectHumidity();
    PTU_DisableTemperatureMeasure();
    rawdata->temperature_humi = _ptu_frequency.FrequencyHz;
    // Main sensor temperature
    PTU_EnableTemperatureMeasure();
    PTU_TemperatureSelectSensor();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_TemperatureDeselectSensor();
    PTU_DisableTemperatureMeasure();
    rawdata->temperature_sensor = _ptu_frequency.FrequencyHz;
    // REF2 1100 ohm resistor
    PTU_EnableTemperatureMeasure();
    PTU_TemperatureSelectREF2();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_TemperatureDeselectREF2();
    PTU_DisableTemperatureMeasure();
    rawdata->temperature_ref2 = _ptu_frequency.FrequencyHz;
}

/**
 * @brief PTU_HumidityMeasure
 * @param rawdata
 */
void PTU_MeasureHumidity(PTURAWData *rawdata) {
    // REF1 capacitor
    PTU_EnableHumidityMeasure();
    PTU_HumiditySelectREF1();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_HumidityDeselectREF1();
    PTU_DisableHumidityMeasure();
    rawdata->humidity_ref1 = _ptu_frequency.FrequencyHz;
    // Main sensor humidity
    PTU_EnableHumidityMeasure();
    PTU_HumiditySelectSensor();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_HumidityDeselectSensor();
    PTU_DisableHumidityMeasure();
    rawdata->humidity_sensor = _ptu_frequency.FrequencyHz;
    // REF2 capacitor
    PTU_EnableHumidityMeasure();
    PTU_HumiditySelectREF2();
    PTU_FrequencyMeasure(&_ptu_frequency);
    PTU_HumidityDeselectREF2();
    PTU_DisableHumidityMeasure();
    rawdata->humidity_ref2 = _ptu_frequency.FrequencyHz;
}