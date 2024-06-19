#include <avr/io.h>
#include <stdint.h>
#include "spi.h"

/*******************************************************************************
*   spi.c v1 - 11/02/2018
        Initial definitions
*   spi.c v2 - 11/07/2018
        Updated initialization functions
*******************************************************************************/

void SPI_init(void) {
	DDRB |= (1 << PB5) | (1 << PB7);
	DDRB &= ~(1 << PB6);
    // set SPI params
	SPCR = (1 << SPE) | (1 << MSTR);
	SPSR = 1 << SPI2X;
}

uint8_t SPI_transfer(uint8_t x) {
		SPDR = x;
		while(!(SPSR & (1<<SPIF)));
		return SPDR;
}
