#ifndef _uart_h
#define _uart_h
#include <avr/pgmspace.h>

/************************************
*   uart.h v1 - 11/02/2018
************************************/
// UART functions
void UART_init(void);
void UART_putc(const unsigned char data);
void UART_puts(const char* charString);
unsigned char UART_getc(void);
unsigned char UART_available(void);

#endif
