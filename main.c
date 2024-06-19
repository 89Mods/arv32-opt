/*
 * arv32-opt: mini-rv32ima on Arduino UNO, but the code is written in pure C
 * Created by gvl610
 * mini-rv32ima is created by cnlohr. See https://github.com/cnlohr/mini-rv32ima
 * UART, SPI, and SD code is created by ryanj1234. See https://github.com/ryanj1234/SD_TUTORIAL_PART4
 */

// UART baudrate. Default is 9600
#define BAUD_RATE 115200

// Headers
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "spi.h"
#include "ddr2.h"

// mini-rv32ima variables
int fail_on_all_faults = 0;
uint64_t lastTime = 0;
struct MiniRV32IMAState *core;

// Functions prototype
static uint32_t HandleException( uint32_t ir, uint32_t retval );
static uint32_t HandleControlStore( uint32_t addy, uint32_t val );
static uint32_t HandleControlLoad( uint32_t addy );
static void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value );
static int32_t HandleOtherCSRRead( uint8_t * image, uint16_t csrno );

// Load / store helper
static uint32_t store4(uint32_t ofs, uint32_t val);
static uint16_t store2(uint32_t ofs, uint16_t val);
static uint8_t store1(uint32_t ofs, uint8_t val);

static uint32_t load4(uint32_t ofs);
static uint16_t load2(uint32_t ofs);
static uint8_t load1(uint32_t ofs);

static uint32_t loadi(uint32_t ofs);

// Other
extern int __heap_start;
extern int *__brkval;
uint32_t last_cyclel = 0; // Last cyclel value
void dump_state(void);

// Config
const uint32_t RAM_SIZE = 16777216UL; // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define DTB_SIZE 1536               // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 1024        // Number of instructions executed before checking status. See loop()
#define TIME_DIVISOR 2

// Setup mini-rv32ima
// This is the functionality we want to override in the emulator.
// think of this as the way the emulator's processor is connected to the outside world.
#define MINIRV32WARN( x... ) UART_pputs( x );
#define MINIRV32_DECORATE  static
#define MINI_RV32_RAM_SIZE RAM_SIZE
#define MINIRV32_IMPLEMENTATION // Minimum rv32 emulator
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { puts("FAULT\r\n"); return 3; } else retval = HandleException( ir, retval ); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( image, csrno, value );
#define MINIRV32_OTHERCSR_READ( csrno, value ) value = HandleOtherCSRRead( image, csrno );
#define MINIRV32_CUSTOM_MEMORY_BUS // Custom RAM handler for swapping to SD card

// Macro for accessing RAM
#define MINIRV32_STORE4( ofs, val ) store4(ofs, val)
#define MINIRV32_STORE2( ofs, val ) store2(ofs, val)
#define MINIRV32_STORE1( ofs, val ) store1(ofs, val)
#define MINIRV32_LOAD4( ofs ) load4(ofs)
#define MINIRV32_LOAD2_SIGNED( ofs ) (int8_t)load2(ofs)
#define MINIRV32_LOAD2( ofs ) load2(ofs)
#define MINIRV32_LOAD1_SIGNED( ofs ) (int8_t)load1(ofs)
#define MINIRV32_LOAD1( ofs ) load1(ofs)
#define MINIRV32_LOADI( ofs ) loadi(ofs)

#include "mini-rv32ima.h"

// millis implementation from https://gist.github.com/adnbr/2439125
volatile uint32_t timer1_millis;
uint32_t last_ms = 0;
 
ISR (TIMER1_COMPA_vect) {
    timer1_millis++;
}

unsigned long millis(void) {
    unsigned long millis_return;

    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        millis_return = timer1_millis;
    }

    return millis_return;
}

void error_out(void) {
		while(1) {
			PORTD ^= 1 << PD7;
			_delay_ms(500);
		}
}

// Entry point
int main(void) {
    // Initialize UART
    UART_init();
    
    //Initialize RAM module
    ddr2_init();

    // Say something, so people know that UART works
    printf("arv32-opt: mini-rv32ima on AtMega32\r\n");

    // Initialize emulator struct
    core = (struct MiniRV32IMAState *)malloc(sizeof(struct MiniRV32IMAState));
    memset(core, 0, sizeof(struct MiniRV32IMAState));

    // Setup core
    core->pc = MINIRV32_RAM_IMAGE_OFFSET;
    core->regs[10] = 0x00; //hart ID
    core->regs[11] = RAM_SIZE - sizeof(struct MiniRV32IMAState) - DTB_SIZE + MINIRV32_RAM_IMAGE_OFFSET; // dtb_pa (Must be valid pointer) (Should be pointer to dtb)
    core->extraflags |= 3; // Machine-mode.

    // Init timer (from https://gist.github.com/adnbr/2439125)
    TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Clock/8
    OCR1A = 2500;
    TIMSK |= (1 << OCIE1A); // Enable the compare match interrupt
    sei(); // Now enable global interrupts

    // Print current free memory
    printf("Current AVR free memory: %u bytes\r\n", (int) SP - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));

    // Emulator loop
    // It's stated in the AVR documentation that writing do ... while() is faster than while()
    do {
        //Disabled bc running out of pins
        /*if (!(PINB & (1 << PINB1))) {
            // Calculate effective emulated speed
            unsigned long current_ms = millis();
            UART_pputs("Effective emulated speed: ");
            UART_putdec32(((core->cyclel - last_cyclel) * 1000) / (current_ms - last_ms));
            UART_pputs(" Hz, dtime=");
            UART_putdec32(current_ms - last_ms);
            UART_pputs("ms, dcycle=");
            UART_putdec32(core->cyclel - last_cyclel);
            UART_pputs("\r\n");

            // Print current free memory
            UART_pputs("Current AVR free memory: ");
            UART_putdec32((int) SP - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
            UART_pputs(" bytes\r\n");
            
            // Dump state
            dump_state();
            UART_pputs("Dump completed. Emulator will continue when B1 is set back to HIGH\r\n");

            // Wait until B1 is set back to HIGH
            while (!(PINB & (1 << PINB1)));
            UART_pputs("B1 is set to HIGH, emulator resume\r\n");
            
            // Reset counters
            last_cyclel = core->cyclel;
            last_ms = millis();
        }*/

        // Calculate pseudo time
        uint64_t * this_ccount = ((uint64_t*)&core->cyclel);
        uint32_t elapsedUs = 0;
        elapsedUs = *this_ccount / TIME_DIVISOR - lastTime;
        lastTime += elapsedUs;

        int ret = MiniRV32IMAStep( core, NULL, 0, elapsedUs, INSTRS_PER_FLIP ); // Execute upto INSTRS_PER_FLIP cycles before breaking out.
        switch( ret )
        {
            case 0: break;
            case 1:
                //_delay_ms(1); //???
                *this_ccount += INSTRS_PER_FLIP;
                break;
            //case 3: instct = 0; break;
            //case 0x7777: goto restart;  //syscon code for restart
            case 0x5555: printf("POWEROFF\r\n"); while(1); //syscon code for power-off . halt
            default: printf("Unknown failure\r\n"); break;
        }
    }while(1);
    printf("I shouldn’t be here...\r\n");

    // Should not get here
    while(1);
}

// Exception handlers
static uint32_t HandleException( uint32_t ir, uint32_t code )
{
	//printf("Handling exception %lx at %lx\r\n", code, ir);
	// Weird opcode emitted by duktape on exit.
	if( code == 3 )
	{
		// Could handle other opcodes here.
	}
	return code;
}

static uint32_t HandleControlStore( uint32_t addy, uint32_t val )
{
	//printf("Control store %lx = %lx\r\n", addy, val);
	if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
	{
		UART_putc(val);
	}
	return 0;
}


static uint32_t HandleControlLoad( uint32_t addy )
{
	//printf("Control store %lx\r\n", addy);
	// Emulating a 8250 / 16550 UART
	if( addy == 0x10000005 )
		return 0x60 | UART_available();
	else if( addy == 0x10000000 && UART_available() )
		return UART_getc();
	return 0;
}

static void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value )
{
	//printf("CSR Write %x = %lx\r\n", csrno, value);
	if( csrno == 0x136 )
	{
		printf("%ld", value);
	}
	if( csrno == 0x137 )
	{
		printf("%08lx", value);
	}
	else if( csrno == 0x138 )
	{
		//Print "string"
		uint32_t ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
		uint32_t ptrend = ptrstart;
		if( ptrstart >= RAM_SIZE ) {
            printf("DEBUG PASSED INVALID PTR(%lu)\r\n", value);
        }
		while( ptrend < RAM_SIZE )
		{
			if( load1(ptrend) == 0 ) break;
			ptrend++;
		}
		if( ptrend != ptrstart ) {
            for (; ptrstart <= ptrend; ptrstart++) {
                UART_putc(load1(ptrstart));
            }
        }
	}
	else if( csrno == 0x139 )
	{
		UART_putc((uint8_t)value);
	}
}

static int32_t HandleOtherCSRRead( uint8_t * image, uint16_t csrno )
{
	//printf("CSR Read %x\r\n", csrno);
	if( csrno == 0x140 )
	{
		if( !UART_available() ) return -1;
		return UART_getc();
	}
	return 0;
}

// Memory access functions
static uint32_t loadi(uint32_t ofs) {
	//printf("loadi %lx\r\n", ofs);
    uint32_t res;
    ddr2_read(ofs, (uint8_t*)&res, 4);
    return res;
}

static uint32_t load4(uint32_t ofs) {
	//printf("load4 %lx\r\n", ofs);
    uint32_t res;
    ddr2_read(ofs, (uint8_t*)&res, 4);
    return res;
}

static uint16_t load2(uint32_t ofs) {
	//printf("load2 %lx\r\n", ofs);
    uint16_t res;
    ddr2_read(ofs, (uint8_t*)&res, 2);
    return res;
}
static uint8_t load1(uint32_t ofs) {
	//printf("load1 %lx\r\n", ofs);
    uint8_t res;
    ddr2_read(ofs, &res, 1);
    return res;
}

static uint32_t store4(uint32_t ofs, uint32_t val) {
	//printf("store4 %lx = %lx\r\n", ofs, val);
    ddr2_write(ofs, (uint8_t*)&val, 4);
    return val;
}

static uint16_t store2(uint32_t ofs, uint16_t val) {
	//printf("store2 %lx = %x\r\n", ofs, val);
    ddr2_write(ofs, (uint8_t*)&val, 2);
    return val;
}

static uint8_t store1(uint32_t ofs, uint8_t val) {
	//printf("store1 %lx = %x\r\n", ofs, val);
    ddr2_write(ofs, &val, 1);
    return val;
}
/*
void dump_state(void) {
    UART_pputs("==============================================================================\r\n");
    UART_pputs("Dumping emulator state:\r\nRegisters x0 - x31:\r\n");

    // Print registers
    for (uint8_t i = 0; i < 8; i++) {
        // Print 4 registers at once
        UART_puthex32(core->regs[i*4]);     UART_pputs(" ");
        UART_puthex32(core->regs[i*4 + 1]); UART_pputs(" ");
        UART_puthex32(core->regs[i*4 + 2]); UART_pputs(" ");
        UART_puthex32(core->regs[i*4 + 3]); UART_pputs("\r\n");
    }

    UART_pputs("pc: ");
    UART_puthex32(core->pc);
    UART_pputs("\r\nmstatus: ");
    UART_puthex32(core->mstatus);
    UART_pputs("\r\ncyclel: ");
    UART_puthex32(core->cyclel);
    UART_pputs("\r\ncycleh: ");
    UART_puthex32(core->cycleh);
    UART_pputs("\r\ntimerl: ");
    UART_puthex32(core->timerl);
    UART_pputs("\r\ntimerh: ");
    UART_puthex32(core->timerh);
    UART_pputs("\r\ntimermatchl: ");
    UART_puthex32(core->timermatchl);
    UART_pputs("\r\ntimermatchh: ");
    UART_puthex32(core->timermatchh);
    UART_pputs("\r\nmscratch: ");
    UART_puthex32(core->mscratch);
    UART_pputs("\r\nmtvec: ");
    UART_puthex32(core->mtvec);
    UART_pputs("\r\nmie: ");
    UART_puthex32(core->mie);
    UART_pputs("\r\nmip: ");
    UART_puthex32(core->mip);
    UART_pputs("\r\nmepc: ");
    UART_puthex32(core->mepc);
    UART_pputs("\r\nmtval: ");
    UART_puthex32(core->mtval);
    UART_pputs("\r\nmcause: ");
    UART_puthex32(core->mcause);
    UART_pputs("\r\nextraflags: ");
    UART_puthex32(core->extraflags);
    UART_pputs("\r\n==============================================================================\r\n");
}
*/
