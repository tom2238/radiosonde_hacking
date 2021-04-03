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

// Define USART console, use USART3 for Vaisala RS41
#define CONSOLE_UART USART3

void console_putc(char c);
char console_getc(int wait);
void console_puts(char *s);
int console_gets(char *s, int len);
void console_print_int(int number);
void console_print_float(float number);
void ftoa(float n, char* res, int afterpoint);

#endif // UTILS_H
