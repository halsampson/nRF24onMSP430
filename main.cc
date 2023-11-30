#include <msp430.h>

#include <stdlib.h>
#include <string.h>

#include <nRF24lite.h>
#include <clocks.h>

#include <private.txt> // RFch, RFaddr

// TODO: 3.3V better than 3.6V regulator?

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

bool diagMode;

void checkSwitches() {
  if (!(P4IN & S1)) {
    setPAlevel(getPAlevel() - 1);
  	setLEDlevel(getPAlevel() << 2);   // red = higher
  	delay(500);
  }

  if (!(P4IN & S2)) {
  	if ((diagMode = !diagMode))
  		setLEDlevel(15); // red - on
  	else setLEDlevel(0); // green - off
  	delay(250);
  }

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

#define ADC_PORT ((PortB*)(P6_BASE + 1))
#define ADC_CH   0                       // A0: JP10-4  or  A1: JP10-6 next to Gnd, blown shorted on 1st proto board
#define ADC_PIN  (1 << ADC_CH)


word readADC() {
  ADC12CTL0 |= ADC12ENC | ADC12SC;
	__bis_SR_register(LPM3_bits + GIE);  // sleep
	return ADC12MEM0;
}

struct {
	word adcNow;
	word adcMin;
	word adcMax;
	word retries; // total
	byte ID;
} payload;
// Note: packet overhead is 9 bytes

bool away;

bool transmit() {
	static byte reconnectWait;
  if (away && ++reconnectWait) // slow retries to 512 seconds when away
  	return false;

  byte sendTries = 4;
  while (sendTries--) {
		payload.retries += read_register(OBSERVE_TX) & 0xF;
		if (write(&payload, sizeof(payload))) {
			away = false; // back reconnected OK
			return true;  // WDT reset here if can't send
		}
  }
  away = true;
  return false;
}

void adcLogging() {
  const byte ReportSecs = 2;
  const byte SampleHz = 60;

  payload.ID = 'C'; // 'E'  'S'

  ADC_PORT->SEL |= ADC_PIN; // A0
  ADC12CTL0 &= ADC12ENC;
  ADC12CTL1 = ADC12SHP | ADC12SSEL_0;  // ADC12OSC = MODOSC ~ 4.8 MHz
  ADC12CTL2 = ADC12TCOFF | ADC12RES_2 | ADC12REFBURST; // 12 bit
  ADC12IE = ADC12IE0;

  // ADC setup to switch to read 12V - in loop ifdef CALIB
  ADC12CTL0 = ADC12SHT0_2 | ADC12ON; // input impedance 38KΩ * 25pF * ln(13) + 800ns = 3.2us * 4.8 MHz = 16 clocks
  ADC12MCTL0 = ADC12EOS | ADC12SREF_0 | ADC12INCH_0 + ADC_CH; // An / AVcc

	payload.adcMin = 0xFFFF;

	while (1) {
	  WDTCTL = WDTPW | WDTSSEL_2 | WDTCNTCL | WDTIS_4; // VLO 14kHz max / 2^15 > 2.3s, typ. 3.5s
		P2DIR &= ~(LEDCath | LEDAnod); // LEDs off

	  long sum = 0;
	  for (byte j = ReportSecs * SampleHz; j--;) {
	  	word sum16 = 0;
	  	for (byte i = 16; i--;) {
	  		sum16 += readADC();
	  		delay_us(1000000 / 16 / SampleHz - (16 + 14) / 4.8 - 0);  // ~60 Hz sum16s; (REFO 0.4% fast)
	  	}

	  	P1OUT ^= BIT1; // JP6-4 30 Hz next to 31.25kHz ACLK / 32 on pin 3

	    if (sum16 < payload.adcMin)
	    	payload.adcMin = sum16;
	    else if (sum16 > payload.adcMax)
	    	payload.adcMax = sum16;

	    sum += sum16;
	  }

	  payload.adcNow = sum / (ReportSecs * SampleHz);

	  // TODO: send calibrated CPU temperature ***

    if (transmit()) {
  		payload.adcMin = 0xFFFF;
  		payload.adcMax = 0;
    } // else accumulate min/max over away trip

	  checkSwitches();

	  if (diagMode && payload.retries > 1) {
	  	setLEDlevel(payload.retries);  // can be > 15
	  	delay(50);  // still low power
	  }
	}
}


// TODO: slower clock, lower Vcore, LPM1+ while ADC sampling using a timer, ..


int main(void) {
	away = SYSRSTIV == SYSRSTIV_WDTTO;  // boot caused by WDT
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initPorts();

	setCPUClockREFO(NomCPUHz);
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS;  // count up at SMCLK

	if (!away)  // not WDT wake
		for (byte level = 0; level < 16; ++level) {
			setLEDlevel(level);
			delay(20);
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
