#include "lpc2103.h"
#include "uart.h"
#include "isr.h"
#include "timer.h"

#define LEDBIT  15

#define IRQ_MASK 0x00000080


void uart0ISR(void) 	  __attribute__ ((interrupt("IRQ")));	// uart.c

void uart0ISR(void) {
  putc(getc());
	VICVectAddr = 0x00000000;	// clear VIC interrupt
}

void enable_uart_interrupts(void) {
  VICIntSelect &= ~(1 << VIC_UART0);  // UART0 selected as IRQ
	VICIntEnable = 1 << VIC_UART0;    // UART0 interrupt enabled
	VICVectCntl1 = VIC_ENABLE | VIC_UART0;
	VICVectAddr1 = (unsigned long) uart0ISR;    // address of the ISR
	U0IER = 0x05;	// enable RBR interrupt 

}


void pll_init(void) {
  /** PLL Calculations:
    PLLCFG 0x42 -> MSEL = 0x00010 (2 - Mult by 3), PSEL = 0x10 (2 - Divide by 4)
    PCLK runs at 60 MHZ
  */ 
  PLLCFG = 0x42;
  PLLCON = 0x01;
  PLLFEED = 0xAA;
  PLLFEED = 0x55;
  while ((PLLSTAT & (1 << 10)) == 0);
  PLLCON = 0x03;
  PLLFEED = 0xAA;
  PLLFEED = 0x55;
  MAMCR = 0x00;
  MAMTIM = 0x03;
  MAMCR = 0x01; 
}

void main(void) {
  SCS = 0x01;
  pll_init();
  APBDIV = 0x01;
  uart_init();
  enable_uart_interrupts();
  initTimer0(0x11e1a300); // IRQ0 1sec; 1msec=0x7530

  while (T0PC < 0x6e1a300);

  enableIRQ();
  IODIR = 1<<15;
  puts("Welcome to my awesome program");
  while (1) {

  }
}
