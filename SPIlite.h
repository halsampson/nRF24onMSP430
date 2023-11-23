#ifndef SPILITE_H_
#define SPILITE_H_

#include <msp430.h>
#include <clocks.h>

typedef unsigned int word;
typedef unsigned char byte;
typedef signed char int8;

typedef struct {
	volatile byte IN, IN_H, Out, Out_H, DIR, DIR_H, REN, REN_H, DS, DS_H, SEL, SEL_H, SEL2, SEL2_H;
} PortB;

#pragma pack(1)

typedef struct {
	union {
		word CTLW0;  // 00h
	  struct {
	    byte CTL1; // 00h
	    byte CTL0; // 01h
	  };
	};
	byte resv2to5[4];
	union {
		word BRW;    // 06h
		struct {
			byte BR0;  // 06h
			byte BR1;  // 07h
		};
	};
	byte MCTL;      // 08h
	byte resv9;
	volatile byte STAT;  // 0Ah
	byte resvB;
	volatile byte RXBUF; // 0Ch
	byte resvD;
	byte TXBUF;           // 0Eh
	byte resvF;
	byte resv10to1B[0xC];
  union {
	  volatile word ICTL;  // 1Ch
	  struct {
	  	byte IE; // 1Ch
	    volatile byte IFG; // 1Dh
	  };
  };
	volatile word IV; // 1Eh
} __attribute__((__packed__)) UCAx;


// port/pin configurations - modify _BASE and pin numbers as needed
#define nRF24SPI ((UCAx*)USCI_A2_BASE)

#define nRF24port ((PortB*)P9_BASE)

#define SCLK    BIT0  // UCA2CLK
#define CSN     BIT1  // SPI
#define CE      BIT2
#define IRQ	     BIT3
#define SDO     BIT4  // UCA2SIMO
#define SDI     BIT5  // UCA2SOMI


void initSPI();

byte xferSPI(byte);


#endif /* SPILITE_H_ */
