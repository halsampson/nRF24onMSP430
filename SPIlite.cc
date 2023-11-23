#include <SPIlite.h>

void initSPI() {
	nRF24port->DIR = SCLK | SDO | CSN | CE;
	nRF24port->DS  = SCLK | SDO | CSN | CE;
	nRF24port->Out = SDI | CSN | IRQ;
	nRF24port->REN = SDI | IRQ; // too slow -- check on board pullup or tri-state?

	nRF24SPI->CTL1 |= UCSWRST;
	nRF24SPI->MCTL = 0;
	nRF24SPI->CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master, 3 pin, synchronous = SPI
	nRF24SPI->CTL1 |= UCSSEL_2; // SMCLK master mode
	nRF24SPI->BR0 = NomCPUHz / 10000000 + 1; // 10 MHz max low cap spec,  20+ MHz typ. works

	nRF24port->SEL = SCLK | SDO | SDI;

	nRF24SPI->CTL1 &= ~UCSWRST;
}

byte xferSPI(byte b) {
	 nRF24port->Out &= ~CSN;  // low = SPI enable/start
   nRF24SPI->TXBUF = b;
   while ( !(nRF24SPI->IFG & UCRXIFG) );  // Wait for RXIFG indicating remote byte received via SOMI
   return nRF24SPI->RXBUF;
}
