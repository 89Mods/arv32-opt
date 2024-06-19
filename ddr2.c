#include <stdint.h>
#include <util/delay.h>
#include <stdio.h>
#include "spi.h"
#include "uart.h"
#include "ddr2.h"

#define READY() ((PIND&8)!=0)
#define WEB (1 << PB0)
#define CEB (1 << PB1)
#define ROM_LEN 16777216UL

/*
 * Memory hardware setup is as follows:
 * 
 * RAM reset line (active low) is at PD6
 * RAM chip-select (active low) is at PB1
 * RAM write-enable (active low) is at PB0
 * RAM Ready output comes in at PD3
 * The 8 bi-directional data lines to the RAM are at PORTC
 * 
 * The RAM module has a 24-bit address bus, which is more pins than there is left
 * Two 74HC573 latches with their inputs wired to PORTA provide IO expansion
 * The first latch is activated by PD2 and buffers address bits 23-16
 * The second latch is activated by PB4 and buffers address bits 15-8
 * The final address bits, 7-0, are wired directly to PORTA
 * 
 * The spiflash ROM uses the normal SPI port pins with its chip-select connected to PB3
 */

void ddr2_init(void) {
    PORTB = 0b1011;
    DDRB = 31;
    SPI_init();
    
    DDRA = 0xFF;
    PORTA = 0;
    DDRC = 0x00;
    PORTC = 0xFF;
	DDRD = 0b11100110;
	PORTD = 128;
    _delay_ms(50);
    //Release reset on memory controller
    PORTD = 64;
    while(!READY()) {}
    puts("DDR2 Ready\r\n");
    
    //Initialize RAM contents from ROM
    PORTB &= ~(1 << PB3);
    SPI_transfer(0x03);
    SPI_transfer(0x00);
    SPI_transfer(0x00);
    SPI_transfer(0x00);
    printf("Writing\r\n");
    for(uint32_t i = 0; i < ROM_LEN; i++) {
        uint8_t val = SPI_transfer(0x00);
        //UART_puthex8(val);
        if((i & 0xFFFFF) == 0) printf("%lu\r\n", i);
        ddr2_write(i, &val, 1);
    }
    PORTB |= 1 << PB3;
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    //Verify
    PORTB &= ~(1 << PB3);
    SPI_transfer(0x03);
    SPI_transfer(0x00);
    SPI_transfer(0x00);
    SPI_transfer(0x00);
    printf("Verifying\r\n");
    for(uint32_t i = 0; i < ROM_LEN; i++) {
		if((i & 0xFFFFF) == 0) printf("%lu\r\n", i);
        uint8_t val = SPI_transfer(0x00);
        uint8_t val2;
        ddr2_read(i, &val2, 1);
        if(val != val2) {
            PORTB |= 1 << PB3;
            PORTD |= 1 << PD7;
            printf("Verify failure at addr %lu, %02x != %02x\r\n", i, val, val2);
            while(1);
        }
    }
    PORTB |= 1 << PB3;
    
    //Patch
    uint8_t val = 0xC8;
    ddr2_write(0xAD, &val, 1);
}

void ddr2_read(uint32_t addr, uint8_t* vals, uint8_t count) {
    DDRC = 0x00;
    PORTC = 0xFF;
    PORTB |= WEB;
    for(uint8_t i = 0; i != count; i++) {
		//Latch address bits
        PORTA = addr>>16;
        PORTD |= 1 << PD2;
        PORTD &= ~(1 << PD2);
        PORTA = addr>>8;
        PORTB |= 1 << PB4;
        PORTB &= ~(1 << PB4);
        //Final address part
        PORTA = addr;
        //Activate
        PORTB &= ~CEB;
		addr++; //This line needs to be right here, doubling as a short delay
		while(!READY()) {}
		//Read result BEFORE releasing CEB
        vals[i] = PINC;
        PORTB |= CEB;
    }
}

void ddr2_write(uint32_t addr, uint8_t* val, uint8_t count) {
    DDRC = 0xFF;
    PORTB &= ~WEB;
    for(uint8_t i = 0; i != count; i++) {
		//Latch address bits
        PORTA = addr>>16;
        PORTD |= 1 << PD2;
        PORTD &= ~(1 << PD2);
        PORTA = addr>>8;
        PORTB |= 1 << PB4;
        PORTB &= ~(1 << PB4);
        //Final address part
        PORTA = addr;
        //Data out
        PORTC = val[i];
        //Activate
        PORTB &= ~CEB;
		addr++; //This line needs to be right here, doubling as a short delay
		while(!READY()) {}
        PORTB |= CEB;
    }
    DDRC = 0x00;
    PORTC = 0xFF;
}
