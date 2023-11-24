#include <msp430.h>

#include <stdlib.h>
#include <string.h>

#include <nRF24lite.h>
#include <clocks.h>


typedef struct {
	volatile unsigned int IN, Out, DIR, REN, DS, SEL[2];

// port 1/2 only
	volatile unsigned int IV[5];
	volatile unsigned int IES, IE, IFG;
} PortW;


void initPorts() {
	for (PortW* p = (PortW*)&PAIN; p <= (PortW*)&PFIN; ++p) {
		p->DIR = p->Out = 0;
		p->REN = 0xFFFF;
	}
}

byte* hexStr(int i) {
  static byte intStr[5+2];
  byte* p = intStr + sizeof(intStr) - 1; // at end
  *p = 0;
  *--p = ' ';
  do {
    unsigned int n = i & 0xF;
    *--p = n <= 9 ? n + '0' : n + 'A' - 10;
  } while (i >>= 4);
  return p;
}


#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
	TA0R;
  P1IFG = 0;
  __bic_SR_register_on_exit (LPM0_bits);
}

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void) {
	TA0R;
  P2IFG = 0;
  __bic_SR_register_on_exit (LPM0_bits);
}

// TODO: 3.3V better than 3.6V regulator
//    best variable level

// TODO: button to cycle power levels for PA, ... test

// TODO: send ADC12 for car monitor; low power sleep mode / wake

// TODO: scan for quiet channels - away from WiFi


#if 0
byte regs[FEATURE + 1];
byte txaddr[5], rxaddr[5];

void dump_registers() {
	for (byte reg = 0; reg <= FEATURE ; ++reg)
		regs[reg] = read_register(reg);
	read_register(TX_ADDR, txaddr, sizeof txaddr);
	read_register(RX_ADDR_P0, rxaddr, sizeof rxaddr);
}
#endif

// Port P2
#define LEDCath BIT0   // JP2-1
#define LEDAnod BIT4   // JP2-2

const byte RFaddr[] = "CarBV";

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initPorts();

	setCPUClockREFO(NomCPUHz);
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS;  // count up at SMCLK

	P2DS = 0;

  initRF24();
  openWritingPipe(RFaddr);

  // dump_registers();

  byte count = 8;
  while (1){
    const byte MaxRetries = 15;
    byte retries = read_register(OBSERVE_TX) & 0xF;

    switch (retries) {
  	case 0 :
  		P2OUT = LEDCath;  // Green bright
  	  P2DIR = LEDCath | LEDAnod;
  	  break;

    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
  	  P2OUT = 0; // off
  	  break;

    default:
  	  P2OUT = P2REN = LEDAnod;  // Red Dim
  	  P2DIR = LEDCath;
  	  break;

    case MaxRetries :
  	  P2OUT = LEDAnod;  // Red bright
  	  P2DIR = LEDAnod | LEDCath;
  	  break;
    }


    char data[ 1 + MaxRetries + 2] = ".................";
    data[count++ % (1 + MaxRetries)] = '\\';
    data[1 + MaxRetries - retries] = '\n';
    data[1 + MaxRetries - retries + 1] = 0;
    write(data, 1 + MaxRetries - retries + 2);

    delay(50);
  }

	return 0;
}

