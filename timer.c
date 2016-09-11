#include "timer.h"
#include "isr.h"
#include "lpc2103.h"
#include "uart.h"

void initTimer0(int cycles) {
	T0PR = cycles >> 1;
	T0MR0 = 1;
	T0MCR = 3;
	T0TCR = 1;
	VICVectAddr0 = (unsigned long)timer0ISR;
	VICVectCntl0 = 0x20 | 4;
	VICIntEnable = 1<<4;
}

void timer0ISR(void) {
	putc('.');
	T0IR = 0x1;
	VICVectAddr = 0;
}
