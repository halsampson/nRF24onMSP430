#include <msp430.h>

#include <stdlib.h>
#include <string.h>

#include <nRF24lite.h>
#include <clocks.h>

#include <private.txt> // RFch, RFaddr

// TODO: send ADC12 for car monitor; low power sleep mode / wake
//   also send retries to check

// TODO: 3.3V better than 3.6V regulator
//    best variable level


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


byte regs[FEATURE + 1];
byte txaddr[5], rxaddr[5];

void dump_registers() {
	for (byte reg = 0; reg <= FEATURE ; ++reg)
		regs[reg] = read_register(reg);
	read_register(TX_ADDR, txaddr, sizeof txaddr);
	read_register(RX_ADDR_P0, rxaddr, sizeof rxaddr);
}


// Port P2
#define LEDCath BIT2  // JP2-5  TA1.1
#define LEDAnod BIT3  // JP2-7  TA1.2


void setLEDlevel(byte level) { // 0: bright green .. 15: bright red
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
	level &= 0xF;
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
    setPAlevel(getPAlevel() - 1);
  	setLEDlevel(getPAlevel() << 2);   // red = higher
  	delay(500);
  	while (!(P4IN & S1)); // wait for switch open
  }

  if (!(P4IN & S2)) WDTCTL = 0; // restart
}


void testTx() {
  dump_registers();

  byte seq;
  while (1){
		const byte MaxRetries = 15;
    byte retries = read_register(OBSERVE_TX) & 0xF;
    setLEDlevel(retries);

    char data[ 1 + MaxRetries + 2] = ".................";
    data[seq++ % (1 + MaxRetries)] = '\\';
    data[1 + MaxRetries - retries] = '\n';
    data[1 + MaxRetries - retries + 1] = 0;
    write(data, 1 + MaxRetries - retries + 2);

    checkSwitches();
    delay(50);
  }
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC_12(void) {
	ADC12IFG = 0;
  __bic_SR_register_on_exit (LPM3_bits);
}

#define ADC_PORT ((PortB*)P1_BASE)
#define ADC_PIN  BIT1

word readADC() {
  ADC12CTL0 |= ADC12ENC | ADC12SC;
	__bis_SR_register(LPM3_bits + GIE);  // sleep
	return ADC12MEM0;
}




void adcLogging() {

	struct {
		word adcNow;
		word adcMin;
		word adcMax;

#ifdef CALIB
		word adcCal;   // occasionally read 2.5V (slow, extra power) with Vdd ref (fast)
		word adcGain;
		word adcOfs;
		word adc25Ref;
#endif
		word retries;  // total since last successful ACKed packet
		byte ID;
	} payload;
	// Note: packet overhead is 9 bytes

	P2DIR &= ~(LEDCath | LEDAnod); // LEDs off

  payload.ID = 'C';

#ifdef CALIB  // resistor divider calibration also needed, so combine
  payload.adcGain = *(word*)0x1A16;
  payload.adcOfs  = *(word*)0x1a18;
  payload.adc25Ref  = *(word*)0x1a2C;
#endif

  ADC_PORT->SEL |= ADC_PIN;
  ADC12CTL0 &= ADC12ENC;
  ADC12CTL1 = ADC12SHP |  ADC12DIV_3 | ADC12SSEL_0;  // ADC12OSC = MODOSC
  ADC12CTL2 = ADC12TCOFF | ADC12RES_2 | ADC12REFBURST; // 12 bit
  ADC12IE = ADC12IE0;

  P6SEL |= BIT0; // A0

#ifdef CALIB
  P5SEL |= BIT0; // A8
  REFCTL0 = REFMSTR | REFVSEL_2 | REFTCOFF | REFOUT | REFON; // 2.5V
#endif

	while (1) {
	  WDTCTL = WDTPW | WDTSSEL_2 | WDTCNTCL | WDTIS_4; // VLO 14kHz max / 2^15 > 2.3s

		payload.adcMin = 0xFFFF;
		payload.adcMax = 0;

	  // ADC setup to switch to read 12V
	  ADC12CTL0 &= ADC12ENC;
	  ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
	  ADC12MCTL0 = ADC12EOS | ADC12SREF_0 | ADC12INCH_0; // A0 / AVcc
	  // 50K input impedance -> 50KΩ * 25pF * ln(13) + 800ns = 4us = 16 clocks

	  for (word sample = 60000; sample; --sample) {
	    payload.adcNow = readADC();
	    if (payload.adcNow < payload.adcMin)
	    	payload.adcMin = payload.adcNow;
	    else if (payload.adcNow > payload.adcMax)
	    	payload.adcMax = payload.adcNow;
	  }

#ifdef CALIB
	  ADC12CTL0 &= ADC12ENC;
	  ADC12CTL0 = ADC12SHT0_2 | ADC12REF2_5V | ADC12REFON | ADC12ON; // 75us settle = 300 clocks (384)
	  ADC12MCTL0 = ADC12EOS | ADC12SREF_0 | ADC12INCH_8; // Vref / Vcc
	  payload.adcCal = readADC();
#endif

	  payload.retries = 0;
	  bool sent;
	  do {
	    sent = write(&payload, sizeof(payload));
	    payload.retries += read_register(OBSERVE_TX) & 0xF;
	  } while (!sent);
	}

}


int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initPorts();

	setCPUClockREFO(NomCPUHz);
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS;  // count up at SMCLK

  for (byte level = 0; level < 16; ++level) {
    setLEDlevel(level);
    delay(50);
  }

  initRF24();

#if 0  // used spectrum scan
  while ((P4IN & S2) && scanChannels() < 255);
  while (1);
#endif

  setChannel(RFch);  // quietest

  openWritingPipe(RFaddr);

#if 0
  testTx();
#endif

  adcLogging();

	return 0;
}

