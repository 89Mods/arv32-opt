#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdio.h>
#include "uart.h"

const char hex_digits[] = "0123456789ABCDEF";

static int uart_putchar(char c, FILE *stream){
	
	while(!(UCSRA & (1<<UDRE)));	//busy loop
	UDR = c;
	
	return 0;
}

//get char over UART (currently not needed)
static int uart_getchar(FILE *stream){
	
	return _FDEV_EOF;
}

void UART_init() {
    #ifndef BAUD_RATE
    #define BAUD_RATE 9600
    #endif

	// set rate
	UBRRL = 129;
	UBRRH = 0;
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0) | (1 << URSEL);
	UCSRB = (1 << TXEN) | (1 << RXEN);
	
	//Define stream for printf
	{
		static FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
		stdout = stderr = &uart_str;
	}
}

void UART_putc(const unsigned char data) {
	// wait for empty transmit buffer
	while(!(UCSRA & (1<<UDRE)));

	// send data to output register
	UDR = data;
}

void UART_puts(const char* charString) {
	// iterate through string
	while(*charString)
		// print character
		UART_putc(*charString++);
}

unsigned char UART_getc(void) {
	// wait for data to be received
	while(!(UCSRA & (1 << RXC)));

	// get data to output register
	return UDR;
}

unsigned char UART_available(void) {
	return UCSRA & (1 << RXC);
}
