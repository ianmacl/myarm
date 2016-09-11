#include "lpc2103.h"

//-------------------------------------------------------------------
void uart_init(void) {
  unsigned int ra;

  ra = PINSEL0;
  ra &= (~0xF);
  ra |= 0x5;
  PINSEL0 = ra;
  // 60000000 Hz PCLK 19200 baud
  // dl 0xC3 (divide by 195) Baud 19231 - difference is 31
  U0ACR = 0x00; // no autobaud
  U0LCR = 0x83; // dlab=1; N81
  U0DLL = 0xC3; // dl = 0x0047
  U0DLM = 0x00; // dl = 0x0047
  U0IER = 0x00; // no interrupts
  U0LCR = 0x03; // dlab=0; N81
  U0IER = 0x00; // no interrupts
  // U0FDR,(0x8<<4)|(0x3<<0)); //mul 0x08, div 0x03
  U0FCR = (1 << 2) | (1 << 1) | (1 << 0); // enable and reset fifos
  U0TER = (1 << 7);                       // transmit enable
}

int putc(int ch) {
    if (ch == '\r' || ch == '\n') {
        while (!(U0LSR & 0x20));
        U0THR = '\n';	// C new line
        while (!(U0LSR & 0x20));
        U0THR = '\r';	// keyboard return
    } else {
		while (!(U0LSR & 0x20));
		U0THR = ch;
    }
    return ch;
}

int getc(void) {
	while (!(U0LSR & 0x01));
	return U0RBR;
}

char* puts(char* str) {
	while (*str) {
		putc(*(str++));
	}
	return str;
}

