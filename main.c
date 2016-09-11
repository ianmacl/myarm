#include "lpc2103.h"
#include "uart.h"
#include "isr.h"

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
	UART0_IER = 0x05;	// enable RBR interrupt 

}


void pll_init(void) {
  // P 2 M  3 FCCO 240000000 CCLK  60000000
  // PUT32(PLLCFG, ((2 - 1) << 5) | ((3 - 1) << 0));
  SCB_PLLCFG = 0x42;
  SCB_PLLCON = 0x01;
  SCB_PLLFEED = 0xAA;
  SCB_PLLFEED = 0x55;
  while ((SCB_PLLSTAT & (1 << 10)) == 0)
    continue;
  SCB_PLLCON = 0x03;
  SCB_PLLFEED = 0xAA;
  SCB_PLLFEED = 0x55;
  MAM_CR = 0x00;
  MAM_TIM = 0x03;
  MAM_CR = 0x01; 
}

void main(void) {
  SCB_SCS = 0x01;
  pll_init();
  SCB_APBDIV = 0x01;
  uart_init();
  enable_uart_interrupts();
  //initTimer0(0x01C9C380); // IRQ0 1sec; 1msec=0x7530

  enableIRQ();
  GPIO0_IODIR = 1<<15;
  puts("Welcome to my awesome program");
  while (1) {

  }
}
