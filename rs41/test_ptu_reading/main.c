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

static volatile uint8_t meas_state = 0;
static volatile uint32_t meas_T1 = 0;
static volatile uint32_t meas_T2 = 0;
static volatile uint32_t meas_Ticks = 0;
static volatile uint16_t meas_TIM2_OVC = 0;
static volatile uint32_t meas_Freq = 0;

void tim2_isr(void);

int main(void) {
    // Setup all parts
    // Set correct clock
    clock_setup();
    // LEDs GPIO
    gpio_setup();
    // Systick for delay function
    systick_setup();
    // USART for serial print
    //usart_setup();
    // PTU timer
   // ptu_timer_setup();
    //tim_setup();

    // Set different leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);

    // Select REF1 750 ohm
    //gpio_set(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
    // Temperature activation
    //gpio_set(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);


    //console_puts("Start ...\n");
    uint32_t timcnt;
    unsigned int oc_value = 1;

	while (1) {
        /* Blink the LED on the board. */
        //gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        //gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        /*gpio_set(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
        delay(10);
        gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);
        delay(10);
        gpio_set(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
        delay(10);
        gpio_clear(PTU_TEMP_REF2_GPIO,PTU_TEMP_REF2_PIN);
        */
        //delay(50);

        // Temperature deactivation
        //gpio_clear(PTU_TEMP_ACTIVATION_GPIO,PTU_TEMP_ACTIVATION_PIN);
        // Deselect REF1 750 ohm
        //gpio_clear(PTU_TEMP_REF1_GPIO,PTU_TEMP_REF1_PIN);

        /*console_puts("Freq: ");
        console_print_int(meas_Freq);
        console_puts(" Hz, ");
        timcnt = timer_get_counter(PTU_MEAS_OUT_TIMER);
        console_puts("cnt: ");
        console_print_int(timcnt);
        console_puts("\n");*/

        /* Blink the LED on the board. */
        gpio_toggle(LED_GREEN_GPIO,LED_GREEN_PIN);
        gpio_toggle(LED_RED_GPIO,LED_RED_PIN);

        //delay(1);

        //delay(30);
        //timer_set_oc_value(TIM2,TIM_OC4, oc_value);
        //oc_value = (unsigned int)(oc_value+10);
        //if(oc_value > 240) {
        //    oc_value = 1;
        //}
	}

	return 0;
}

void tim2_isr(void) {
    if (timer_get_flag(PTU_MEAS_OUT_TIMER, TIM_SR_CC2IF)) {
        if(meas_state == 0) {
            meas_T1 = timer_get_counter(PTU_MEAS_OUT_TIMER);
            meas_TIM2_OVC = 0;
            meas_state = 1;
        } else {
            meas_T2 = timer_get_counter(PTU_MEAS_OUT_TIMER);
            meas_Ticks = (meas_T2 + (meas_TIM2_OVC * 65536)) - meas_T1;
            meas_Freq = (uint32_t)(rcc_ahb_frequency/meas_Ticks);
            meas_state = 0;
        }
        timer_clear_flag(PTU_MEAS_OUT_TIMER, TIM_SR_CC2IF); /* Clear interrupt flag. */
    }
    console_putc('I');
}
