#ifndef UART_H
#define UART_H


#define CONSOLE_UART	USART3

void console_putc(char c);
char console_getc(int wait);
void console_puts(char *s);
int console_gets(char *s, int len);

#endif // UART_H
