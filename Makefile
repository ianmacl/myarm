ARMGNU?=arm-none-eabi
AOPS = --warn --fatal-warnings --alternate
COPS = -Wall -Werror -O2 -nostdlib -nostartfiles -ffreestanding
LLCOPS = 
OOPS = -std-compile-opts

all : main.hex

install : main.hex
	lpc21isp -control -term main.hex $(USB_PORT) 19200 20000

startup.o : startup.s
	$(ARMGNU)-as $(AOPS) startup.s -o startup.o

main.hex : linker.ld startup.o uart.o main.o isr.o
	$(ARMGNU)-ld -T linker.ld startup.o main.o uart.o isr.o -o main.elf
	$(ARMGNU)-objdump -D main.elf > main.list
	$(ARMGNU)-objcopy main.elf main.hex -O ihex

isr.o : isr.c
	$(ARMGNU)-gcc $(COPS) -c isr.c -o isr.o

uart.o : uart.c
	$(ARMGNU)-gcc $(COPS) -c uart.c -o uart.o

main.o : main.c
	$(ARMGNU)-gcc $(COPS) -c main.c -o main.o

clean:
	rm -f *.hex
	rm -f *.o
	rm -f *.elf
	rm -f *.list
