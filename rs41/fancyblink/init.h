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

void gpio_setup(void);

void clock_setup(void);

void systick_setup(void);

uint64_t millis(void);

void delay(uint64_t duration);

#endif // STM32_INIT_C
