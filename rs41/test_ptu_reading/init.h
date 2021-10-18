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

#ifndef STM32_INIT_C
#define STM32_INIT_C

// Vaisala RS41 leds
#define LED_GREEN_GPIO GPIOB
#define LED_GREEN_PIN GPIO7
#define LED_GREEN_RCC RCC_GPIOB
#define LED_RED_GPIO GPIOB
#define LED_RED_PIN GPIO8
#define LED_RED_RCC RCC_GPIOB

// Vaisala RS41 XDATA UART
#define XDATA_USART USART3
#define XDATA_USART_GPIO GPIOB
#define XDATA_USART_RCC_GPIO RCC_GPIOB
#define XDATA_USART_RCC RCC_USART3

// Vaisala RS41 external timer input
#define PTU_MEAS_OUT_TIMER TIM2
#define PTU_MEAS_OUT_TIMER_RCC RCC_TIM2
#define PTU_MEAS_OUT_TIMER_IRQ NVIC_TIM2_IRQ
#define PTU_MEAS_OUT_TIMER_RST RST_TIM2
#define PTU_MEAS_OUT_TIMER_PERIOD 0xFFFF

// TIM3 counter as stopwatch
#define FREQUENCY_TIMER TIM3
#define FREQUENCY_TIMER_RCC RCC_TIM3
#define FREQUENCY_TIMER_IRQ NVIC_TIM3_IRQ
#define FREQUENCY_TIMER_RST RST_TIM3
#define FREQUENCY_TIMER_PERIOD 0xFFFF
#define FREQUENCY_TIMER_CLOCK 24000000UL
#define FREQUENCY_TIMER_PULSE_LIMIT 100

// Vaisala RS41 PTU pins
// Temperature
#define PTU_TEMP_REF1_GPIO GPIOA
#define PTU_TEMP_REF1_PIN GPIO3
#define PTU_TEMP_REF1_RCC RCC_GPIOA
#define PTU_TEMP_REF2_GPIO GPIOB
#define PTU_TEMP_REF2_PIN GPIO6
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

void gpio_setup(void);
void ptu_timer_setup(void);
void usart_setup(void);
void clock_setup(void);
void systick_setup(void);

#endif // STM32_INIT_C
