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
#include "ptu_measure.h"

// Original sonde data
// Tc: 179876   132438   192018
// Th: 188669   132438   192019
// Rh: 553198   483470   551322
// calT1: 1.303953 ;  -0.095812 ;   0.005378 ;
// calH: 44.937469 ;   5.023599 ;
// calT2: 1.265843 ;   0.122289 ;   0.005889 ;
/*
[  834] (T0351185) Do 2021-10-28 19:27:47.013  lat: 43.68664  lon: -61.34227  alt: 10157.60   vH:  0.0  D:   0.0  vV: 0.0   T=22.4C  RH=47%
  h: 10157.60   # 1:   179876   132438   192018   #   2:   553198   483470   551322   #   3:   188669   132438   192019   #      Tc:22.58  RH:46.3  TH:28.33
     10157.60 ;  750.0 ; 1100.0 ;   1.303953 ;  -0.095812 ;   0.005378 ;  44.937469 ;   5.023599 ;   1.265843 ;   0.122289 ;   0.005889
*/

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
    // Inicialize PTU
    PTU_Init();

    Si4032_DisableTx();
    PTU_DisableReferenceHeating();

    // Set leds state
    gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
    gpio_clear(LED_RED_GPIO,LED_RED_PIN);

    console_puts("Start ...\n");

    PTURAWData raw_ptu;
    PTUCalculatedData calculated_ptu;
    const PTUCalibrationData calib_ptu = {.cal_T1 = {1.303953f, -0.095812f, 0.005378f}, .cal_H = {44.937469f, 5.023599f}, .cal_T2 = {1.265843f, 0.122289f, 0.005889f} };

	while (1) {
        /* Blink the LED on the board. */   
        gpio_clear(LED_RED_GPIO,LED_RED_PIN);

        PTU_MeasureTemperature(&raw_ptu);

        /* Blink the LED on the board. */
        gpio_set(LED_RED_GPIO,LED_RED_PIN);

        console_puts("T_REF1: ");
        console_print_int(raw_ptu.temperature_ref1);
        console_puts(" pulses ");

        console_puts("T_Humidity: ");
        console_print_int(raw_ptu.temperature_humi);
        console_puts(" pulses ");

        console_puts("T_Sensor: ");
        console_print_int(raw_ptu.temperature_sensor);
        console_puts(" pulses ");

        console_puts("T_REF2: ");
        console_print_int(raw_ptu.temperature_ref2);
        console_puts(" pulses\n");

        /* Blink the LED on the board. */
        gpio_clear(LED_RED_GPIO,LED_RED_PIN);

        PTU_MeasureHumidity(&raw_ptu);

        /* Blink the LED on the board. */
        gpio_set(LED_RED_GPIO,LED_RED_PIN);

        console_puts("H_REF1: ");
        console_print_int(raw_ptu.humidity_ref1);
        console_puts(" pulses ");

        console_puts("H_Sensor: ");
        console_print_int(raw_ptu.humidity_sensor);
        console_puts(" pulses ");

        console_puts("H_REF2: ");
        console_print_int(raw_ptu.humidity_ref2);
        console_puts(" pulses\n");

        // Get temperature in celsius
        PTU_CalculateData(&raw_ptu,&calculated_ptu,calib_ptu);
        console_puts("Temperature sensor: ");
        console_print_float(calculated_ptu.temperature_sensor);
        console_puts(" degC ");
        console_puts("Temperature humidity: ");
        console_print_float(calculated_ptu.temperature_humi);
        console_puts(" degC ");
        console_puts("Humidity sensor: ");
        console_print_int((int)calculated_ptu.humidity_sensor);
        console_puts(" % ");
        console_puts("Dew point: ");
        console_print_float(calculated_ptu.dew_point);
        console_puts(" degC\n");

        /* Blink the LED on the board. */
        gpio_clear(LED_GREEN_GPIO,LED_GREEN_PIN);
        delay(800);
        gpio_set(LED_GREEN_GPIO,LED_GREEN_PIN);
	}
	return 0;
}

