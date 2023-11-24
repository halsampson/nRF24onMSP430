#include <msp430.h>

#include <stdlib.h>
#include <string.h>

#include <nRF24lite.h>
#include <clocks.h>


typedef struct {
	volatile word IN, Out, DIR, REN, DS, SEL[2];
	volatile word IV1; // port 1/2 only
	word resv[4];
	volatile word IES, IE, IFG, IV2;
} PortW;

void initPorts() { // all inputs with pullups
	for (PortW* p = (PortW*)&PAIN; p <= (PortW*)&PFIN; ++p) {
		p->DIR = 0;
		p->REN = p->Out = 0xFFFF;
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

#define DUMP_REGISTERS

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
#define LEDCath BIT2  // JP2-5  TA1.1
#define LEDAnod BIT3  // JP2-7  TA1.2


void setLEDlevel(byte level) {
	// Red 1.65V 1mA   Grn 1.9V 1mA
	// REN: 35K -- 60uA very dim
	// DS: ~3:1	 20Ω/60Ω
	// Color Grn Mixed Red

	P2DS = 0;
	P2DIR = P2SEL = LEDCath | LEDAnod;

	TA1CTL = TASSEL_2 | MC_2; // SMCLK Continuous
	TA1CCTL1 = OUTMOD_3; // Cath / hi at CCR1
	TA1CCTL2 = OUTMOD_7; // Anod \ lo at CCR2
	TA1CCR0 = 0xFFFF;

	// starts green
	// toggle both for red

	// logarithmic: 16 steps 0 .. 15:   shift by level
	TA1CCR1 = (word)0xFFFD >> level;  // Green on time if CCR2 > CCR1
	TA1CCR2 = -2 << level;  // Red time from CCR2 (> CCR1) to CCR0

	// CCR1 <= CCR2 <= cCR0 = 0xFFFF
	// Red time = Cath lo, Anod hi = CCR1
	// Grn time = Anod lo, Cath hi = -CCR2
}


// Port P4
#define S1 BIT1
#define S2 BIT2

void checkSwitches() {
  if (!(P4IN & S1)) {
    setPAlevel(getPAlevel() + 1);
  	setLEDlevel(getPAlevel() << 2);   // red = higher
  	delay(500);
  	while (!(P4IN & S1)); // wait for switch open
  }

  if (!(P4IN & S2)) WDTCTL = 0; // restart
}


const byte RFaddr[] = "CarBV";

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initPorts();

	setCPUClockREFO(NomCPUHz);
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS;  // count up at SMCLK

	for (byte level = 0; level < 16; ++level) {
		setLEDlevel(level);
		delay(100);
	}

  initRF24();
  openWritingPipe(RFaddr);

#ifdef DUMP_REGISTERS
   dump_registers();
#endif

  byte count = 8;
  while (1){
    const byte MaxRetries = 15;
    byte retries = read_register(OBSERVE_TX) & 0xF;
    setLEDlevel(retries);

    char data[ 1 + MaxRetries + 2] = ".................";
    data[count++ % (1 + MaxRetries)] = '\\';
    data[1 + MaxRetries - retries] = '\n';
    data[1 + MaxRetries - retries + 1] = 0;
    write(data, 1 + MaxRetries - retries + 2);

    checkSwitches();
    delay(50);
  }

	return 0;
}

