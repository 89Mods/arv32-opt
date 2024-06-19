CC = avr-gcc
LD = avr-gcc

CFLAGS += -mmcu=atmega32 -I. -DF_CPU=20000000UL  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99
LDFLAGS += -mmcu=atmega32 -I. -DF_CPU=20000000UL -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99

OBJS=main.o spi.o uart.o ddr2.o

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

out:	$(OBJS)
	avr-gcc $(LDFLAGS) $^ --output main.elf -Wl,-Map=main.map,--cref
	avr-objcopy -O ihex -R .eeprom main.elf main.hex
	avr-size -A -d main.elf

clean:
	rm -f *.o *.elf *.hex

all: out

.PHONY: out
