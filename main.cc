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


// TODO: fix retries vs. Ard? timing? noise? ???
// TODO: dynamic packet length: enable on rcvr also ***
// TODO: PA/LNA version support

// TODO: serial connection for debugging *****

const byte RFaddr[] = "CarBV";

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initPorts();

	setCPUClockREFO(NomCPUHz);
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS;  // count up at SMCLK

  initRF24();
  openWritingPipe(RFaddr);

  byte count = 0;
  while (1){
    const byte MaxRetries = 15;
    byte retries = read_register(OBSERVE_TX) & 0xF;
    byte data[] = "********************";
    data[count++ % (1 + MaxRetries)] = '.';
    data[1 + MaxRetries - retries] = '\n';
    data[1 + MaxRetries - retries + 1] = 0;
    write(data);

    delay(50);
  }

	return 0;
}

