#include "lpc2103.h"

//-------------------------------------------------------------------
void uart_init(void) {
  unsigned int ra;

  ra = PCB_PINSEL0;
  ra &= (~0xF);
  ra |= 0x5;
  PCB_PINSEL0 = ra;
  // 60000000 Hz PCLK 38400 baud
  // dl 0x47 mul 0x08 div 0x03 baud 38412 diff 12
  UART0_ACR = 0x00; // no autobaud
  UART0_LCR = 0x83; // dlab=1; N81
  UART0_DLL = 0xC3; // dl = 0x0047
  UART0_DLM = 0x00; // dl = 0x0047
  UART0_IER = 0x00; // no interrupts
  UART0_LCR = 0x03; // dlab=0; N81
  UART0_IER = 0x00; // no interrupts
  // U0FDR,(0x8<<4)|(0x3<<0)); //mul 0x08, div 0x03
  UART0_FCR = (1 << 2) | (1 << 1) | (1 << 0); // enable and reset fifos
  UART0_TER = (1 << 7);                       // transmit enable
}

int putc(int ch) {
    if (ch == '\r' || ch == '\n') {
        while (!(UART0_LSR & 0x20));
        UART0_THR = '\n';	// C new line
        while (!(UART0_LSR & 0x20));
        UART0_THR = '\r';	// keyboard return
    } else {
		while (!(UART0_LSR & 0x20));
		UART0_THR = ch;
    }
    return ch;
}

int getc(void) {
	while (!(UART0_LSR & 0x01));
	return UART0_RBR;
}

char* puts(char* str) {
	while (*str) {
		putc(*(str++));
	}
	return str;
}

