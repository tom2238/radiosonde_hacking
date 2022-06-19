/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#if 0

extern void itoa( int n, char s[] ) ;

#else

extern char* itoa( int value, char *string, int radix ) ;
extern char* ltoa( long value, char *string, int radix ) ;
extern char* utoa( unsigned long value, char *string, int radix ) ;
extern char* ultoa( unsigned long value, char *string, int radix ) ;
#endif /* 0 */

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

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

// Define USART console, use USART3 for Vaisala RS41
#define CONSOLE_UART USART3

void console_putc(char c);
char console_getc(int wait);
void console_puts(char *s);
int console_gets(char *s, int len);
void console_print_int(int number);
void console_print_float(float number);
void ftoa(float n, char* res, int afterpoint);
/*
 * Systick delay
 */
uint64_t millis(void);
void delay(uint64_t duration);
extern volatile uint8_t mcu_main_pulse_enable;

#endif // UTILS_H
