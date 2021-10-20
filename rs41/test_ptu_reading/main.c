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

/* QtCreator run options for STM32 kit:
 * Executable: st-flash
 * Command line arguments: write main.bin 0x8000000
 * Working directory: %{CurrentProject:BuildPath}
 */

/* Add
 * ../../libopencm3/include
 * ../libraries
 * into <project name>.include file to show libs and function help
 */

/* Vaisala RS41 Upload
 * 1. Press power button to power up MCU
 * 2. Erase with: st-flash erase
 * 3. Upload with: st-flash wrire main.bin 0x8000000
 * 4. st-info --probe output:
 * Found 1 stlink programmers
 * version:    V2J29S7
 * serial:     550035001100005153484c4e
 * hla-serial: "\x55\x00\x35\x00\x11\x00\x00\x51\x53\x48\x4c\x4e"
 * flash:      65536 (pagesize: 1024)
 * sram:       8192
 * chipid:     0x0420
 * descr:      F1xx Value Line
 * In case of problem, try connect under reset
 */

// Global LibOpenCM3 libraries
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
// STM32F100 libraries
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/timer.h>
// Another libraries
#include <stdint.h>
#include "init.h"
#include "utils.h"
#include "si4032.h"

typedef enum {
    FREQ_STATE_IDLE = 1,
    FREQ_STATE_RUN,
    FREQ_STATE_DONE
}PTUFrequencyCounterState;

typedef struct {
    uint32_t OverCapture;
    uint32_t Pulses;
    uint32_t PeriodTimer;
    uint32_t FrequencyHz;
    PTUFrequencyCounterState State;
}PTUFrequencyCounter;

void tim2_isr(void);
void tim3_isr(void);
void PTU_FrequencyMeasure(PTUFrequencyCounter *fcounter);

PTUFrequencyCounter ptu_meas = {0,0,0,0,FREQ_STATE_IDLE};
uint32_t c_cnt = 0;

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // Systick for delay function
    systick_setup();
    // LEDs GPIO
    gpio_setup();
    // USART for serial print
    usart_setup();
    // SPI config
    spi_setup();
    // Wait at least 15ms before any initialization SPI commands are sent to the radio
    delay(20);
    // Inicialize Si4032
    Si4032_Init();
    // For some reason we have to do this again
    delay(20);
    Si4032_Init2();
    // PTU timer
    ptu_timer_setup();

    Si4032_DisableTx();
    Si4032_GPIOClear(SI4032_GPIO_PORT_1);

    // Set leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);

    // PTU setup
    gpio_set(PTU_HUMI_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
    gpio_set(PTU_TEMP_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
    gpio_set(PTU_HUMI_REF1_GPIO,PTU_HUMI_REF1_PIN);
    gpio_set(PTU_HUMI_REF2_GPIO,PTU_HUMI_REF2_PIN);
    gpio_set(PTU_HUMI_SENSOR_GPIO,PTU_HUMI_SENSOR_PIN);
    gpio_set(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
    gpio_set(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
    gpio_set(PTU_TEMP_SENSOR_GPIO,PTU_TEMP_SENSOR_PIN);
    gpio_set(PTU_TEMP_HUMI_GPIO,PTU_TEMP_HUMI_PIN);
    delay(10);
    gpio_clear(PTU_HUMI_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
    gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_HUMI_ACTIVATION_PIN);
    gpio_clear(PTU_HUMI_REF1_GPIO,PTU_HUMI_REF1_PIN);
    gpio_clear(PTU_HUMI_REF2_GPIO,PTU_HUMI_REF2_PIN);
    gpio_clear(PTU_HUMI_SENSOR_GPIO,PTU_HUMI_SENSOR_PIN);
    gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
    gpio_clear(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
    gpio_clear(PTU_TEMP_SENSOR_GPIO,PTU_TEMP_SENSOR_PIN);
    gpio_clear(PTU_TEMP_HUMI_GPIO,PTU_TEMP_HUMI_PIN);

    console_puts("Start ...\n");

	while (1) {
        /* Blink the LED on the board. */   
        gpio_clear(LED_RED_GPIO,LED_RED_PIN);

        // Select REF1 750 ohm
        gpio_set(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
        // Temperature activation
        gpio_set(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);

        PTU_FrequencyMeasure(&ptu_meas);

        // Temperature deactivation
        gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
        // Deselect REF1 750 ohm
        gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);

        /* Blink the LED on the board. */
        gpio_set(LED_RED_GPIO,LED_RED_PIN);

        console_puts("REF1: Ticks: ");
        console_print_int(ptu_meas.PeriodTimer);
        console_puts(", Freq: ");
        console_print_int(ptu_meas.FrequencyHz);
        console_puts(" Hz\n");

        /* Blink the LED on the board. */
        gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);

        // Select REF2 1100 ohm
        gpio_set(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
        // Temperature activation
        gpio_set(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);

        PTU_FrequencyMeasure(&ptu_meas);

        // Temperature deactivation
        gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
        // Deselect REF2 1100 ohm
        gpio_clear(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);

        /* Blink the LED on the board. */
        gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);

        console_puts("REF2: Ticks: ");
        console_print_int(ptu_meas.PeriodTimer);
        console_puts(", Freq: ");
        console_print_int(ptu_meas.FrequencyHz);
        console_puts(" Hz\n");

        delay(900);

        //console_puts("C:");
        //console_print_int(c_cnt);
        //console_puts("\n");
        c_cnt = 0;
	}
	return 0;
}

void tim2_isr(void) {
    if (timer_get_flag(PTU_MEAS_OUT_TIMER, TIM_SR_CC2IF)) {
        //c_cnt++;
        if (ptu_meas.State == FREQ_STATE_RUN) {
            ptu_meas.Pulses++;
            if(ptu_meas.Pulses > FREQUENCY_TIMER_PULSE_LIMIT) {
                ptu_meas.PeriodTimer = timer_get_counter(FREQUENCY_TIMER);
                ptu_meas.State = FREQ_STATE_DONE;
                timer_disable_counter(FREQUENCY_TIMER);
            }
        }
        timer_clear_flag(PTU_MEAS_OUT_TIMER, TIM_SR_CC2IF);
    }
}

void tim3_isr(void) {
    if(timer_get_flag(FREQUENCY_TIMER, TIM_SR_UIF)) {
        ptu_meas.OverCapture += 65536;
        timer_clear_flag(FREQUENCY_TIMER, TIM_SR_UIF);
    }
}

void PTU_FrequencyMeasure(PTUFrequencyCounter *fcounter) {
    // Timeout in ms
    uint16_t timeout = 100;
    // Reset before measure
    fcounter->State = FREQ_STATE_RUN;
    fcounter->OverCapture = 0;
    fcounter->Pulses = 0;
    fcounter->PeriodTimer = 0;
    fcounter->FrequencyHz = 0;
    // Reset the counter. This will generate one extra overflow for next measurement.
    // In case of nothing got counted, manually generate a reset to keep consistency.
    timer_set_counter(FREQUENCY_TIMER,0);
    // Enable the counter
    timer_enable_counter(FREQUENCY_TIMER);
    // Wait for ticks or timeout
    while(fcounter->Pulses <= FREQUENCY_TIMER_PULSE_LIMIT && timeout-- > 0) {
        delay(1);
    }
    // Timeout, low frequency or short timeout
    // Because if timeout reached, add one into timeout variable to get zero.
    timeout++;
    // Debug
    //console_puts("T:");
    //console_print_int(timeout);
    //console_puts(", P:");
    //console_print_int(fcounter->PeriodTimer);
    //console_puts("\n");
    if(!timeout) {
        fcounter->FrequencyHz = 0;
        fcounter->PeriodTimer = 0;
    } else {
        // Frequency is correct
        fcounter->PeriodTimer += fcounter->OverCapture;
        fcounter->FrequencyHz = (FREQUENCY_TIMER_CLOCK)/(fcounter->PeriodTimer/FREQUENCY_TIMER_PULSE_LIMIT);
    }
}
