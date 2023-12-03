#include <msp430.h>

#include <stdlib.h>
#include <string.h>

#include <nRF24lite.h>
#include <clocks.h>

#include <private.txt> // RFch, RFaddr

// use shielded cable

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

void testTx() { // retries to LEDs for signal propagation walk testing
	for (byte level = 0; level < 16; ++level) {
		setLEDlevel(level);
		delay(20);
	}

	dump_registers();

  byte seq;
  while (1) {
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
  __bic_SR_register_on_exit (LPM4_bits);
}

word readADC() {
	ADC12CTL0 &= ~ADC12ENC;
  ADC12CTL0 |= ADC12ENC | ADC12SC;
	__bis_SR_register(LPM4_bits + GIE);  // sleep
	return ADC12MEM0;
}

#define TAG_ADC12               0x1A14
#define CAL_ADC_GAIN_FACTOR   (*((uint*)(TAG_ADC12 + 2)))     // ~32768
#define CAL_ADC_OFFSET         (*((int*)(TAG_ADC12 + 4)))

#define CAL_ADC_15T30          (*((int*)(TAG_ADC12 + 6)))
#define CAL_ADC_15T85          (*((int*)(TAG_ADC12 + 8)))

#define CAL_ADC_15VREF_FACTOR  (*((unsigned int*)(TAG_ADC12 + 20)))   // ~32768
#define CAL_ADC_20VREF_FACTOR  (*((unsigned int*)(TAG_ADC12 + 22)))   // ~32768
#define CAL_ADC_25VREF_FACTOR  (*((unsigned int*)(TAG_ADC12 + 24)))   // ~32768

int dieTemp() {
  ADC12CTL0 &= ~ADC12ENC;
  ADC12CTL2 = ADC12RES_2 | ADC12REFBURST;
  ADC12CTL0 = ADC12SHT0_10 | ADC12REFON | ADC12ON;  // max(75, 30)us * 5.4Mhz = 405 ADC clocks < 512
  ADC12MCTL0 = ADC12EOS | ADC12SREF_1 | ADC12INCH_10; // Temp diode / REF1_5V

  return ((int)readADC() - CAL_ADC_15T30) * (85 - 30) * 100L / (CAL_ADC_15T85 - CAL_ADC_15T30) + 30 * 100;  // in hundredths of degrees Celsius
    // calibration error +/- 3°C, matched within a wafer
}

word VccDiv5V() {
  ADC12CTL0 &= ~ADC12ENC;
  ADC12CTL0 = ADC12SHT0_2 | ADC12REF2_5V | ADC12REFON | ADC12ON;
  ADC12MCTL0 = ADC12EOS | ADC12SREF_1 | ADC12INCH_11;  // Vcc/2  / REF2.5V

  return readADC() + CAL_ADC_OFFSET;
}

struct {
	word adcNow;  // mV
	word adcMin;
	word adcMax;
	word retries; // total
	char ID[2];
	short degreesC; // in hundredths
} payload;
// Note: packet overhead is 9 bytes

bool away;

bool transmit() {
#if 1  // 0 for quick testing
	static byte reconnectWait;
  if (away && ++reconnectWait) // slow retries to 512 seconds when away
  	return false;
#endif

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

byte unit;

#define ADC_PORT ((PortB*)(P6_BASE + 1))
#define ADC_CH   unit                       // A0: JP10-4 blown on 2n board;  or  A1: JP10-6 next to Gnd, blown shorted on 1st proto board
#define ADC_PIN  (1 << ADC_CH)

const byte NumUnits = 1 + 3;
word rCal[NumUnits] = {47608, 47578, 47895, 47895,};  // 2 * 5 * 1000 * (180 + 47.5) / 47.5, adjusted for Vref, resistors, ...

void adcLogging() {
	const byte SamplesPerCycle = 16;
  const byte Sample16Hz = 60;
  const byte SamplesPerReport = 128;// ~2 sec

  payload.ID[0] = RFaddr[unit][0];
  payload.ID[1] = RFaddr[unit][1];

  ADC_PORT->SEL |= ADC_PIN; // A0
  ADC12CTL0 &= ~ADC12ENC;
  REFCTL0 &= ~REFMSTR;  // use legacy control bits
  ADC12CTL1 = ADC12SHS_2 | ADC12SHP | ADC12SSEL_0 | ADC12CONSEQ_1;  // TB0.0  ADC12OSC = MODOSC ~ 4.8 MHz

  TB0CTL = TBSSEL__ACLK | MC__UP;
  TB0CCTL0 = OUTMOD_4; // toggle, so half frequency
  TB0CCR0 = REFOCLK_HZ / Sample16Hz / SamplesPerCycle / 2 - 1;

  volatile byte* mctl = &ADC12MCTL0;
  for (int i = 0; i < SamplesPerCycle; ++i)
  	*mctl++ = ADC12SREF_0 | ADC12INCH_0 + ADC_CH;  // An / AVcc
  ADC12MCTL15 |= ADC12EOS;

  word adcMin = 0xFFFF;
  word adcMax = 0;

	while (1) {
	  WDTCTL = WDTPW | WDTSSEL_2 | WDTCNTCL | WDTIS_4; // VLO 14kHz max / 2^15 > 2.3s, typ. 3.5s
		P2DIR &= ~(LEDCath | LEDAnod); // LEDs off

	  // ADC setup to switch to read 12V
	  ADC12CTL0 &= ~ADC12ENC;
	  ADC12CTL0 = ADC12SHT1_12 |ADC12SHT0_12 | ADC12ON; // input impedance 38KΩ * 25pF * ln(12 + 1) + 800ns = 3.2us * 5.4 MHz = 18 clocks min ; 1024 better 16 bit
	  ADC12CTL1 = ADC12SHP | ADC12SHS_2 | ADC12SSEL_0 | ADC12CONSEQ_1;  // TB0.0  ADC12OSC = MODOSC ~ 4.8 MHz
	  ADC12CTL2 = ADC12TCOFF | ADC12RES_2 | ADC12REFBURST; // 12 bit
	  ADC12MCTL0 = ADC12SREF_0 | ADC12INCH_0 + ADC_CH; // An / AVcc
	  ADC12IE = ADC12IE15;

	  long sum = 0;
	  for (byte j = SamplesPerReport; j--;) {
	    readADC();
	  	word sum16 = 0;
	  	volatile word* adcmem = &ADC12MEM0;
	  	for (byte i = SamplesPerCycle; i--;)
	  		sum16 += *adcmem++ + CAL_ADC_OFFSET;

	  	P1OUT ^= BIT2; // JP6-5 ~30 Hz next to 31.25kHz ACLK / 32 on pin 3; Gnd pin 1

	    if (sum16 < adcMin)
	    	adcMin = sum16;
	    else if (sum16 > adcMax)
	    	adcMax = sum16;

	    sum += sum16;
	  }

	  payload.adcNow = sum / SamplesPerReport;

	  ADC12IE = ADC12IE0; // for two single samples
	  ADC12CTL1 = ADC12SHP | ADC12SSEL_0;

	  payload.degreesC = dieTemp();

	  // Internal refs typ.     30 ppm/°C
	  //    vs. XC6206 typ. +/-100 ppm/°C
	  //    so calibrate: Vcc / 2 / REF2.5V
	  word vCal = (long)rCal[unit] * VccDiv5V() / 4096;

	  payload.adcNow = (payload.adcNow * (long)vCal) >> 17; // send mV
	  payload.adcMin = (adcMin * (long)vCal) >> 17;
	  payload.adcMax = (adcMax * (long)vCal) >> 17;

	  if (transmit()) {
  		adcMin = 0xFFFF;
  		adcMax = 0;
    } // else accumulate min/max over away trip

	  checkSwitches();

	  if (diagMode && payload.retries > 1) {
	  	setLEDlevel(payload.retries);  // can be > 15
	  	delay(50);  // still low power
	  }
	}
}


int main(void) {
	away = SYSRSTIV == SYSRSTIV_WDTTO;  // boot caused by WDT
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initPorts();

	UCSCTL4 = SELA__REFOCLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV; 	// 32KHz,  1.048576 MHz clocks

	UCSCTL5 = DIVPA__1;  // ACLK / N out
	P1SEL = BIT0;  // 1.05MHz / 32 = 32KHz on JP6-3   (loses FLL lock in LPM3!)
	P1DIR |= BIT0;

  initRF24();

#if 0  // used spectrum scan
  while ((P4IN & S2) && scanChannels() < 255); // overnight
  while (1);
#endif
  setChannel(RFch);  // quietest

  switch (*(word*)0x1A02) {  // CRC of Device Descriptor Table ~ serial
   // case 0x???: unit = 0; break;  // TODO: check
    case 0x8EED: unit = 1; break;  // latest
    default: unit = 0; break;
  }

#if 0
  openWritingPipe(RFaddr[unit]);  // fix
#else
  openWritingPipe(RFaddr[0]);
#endif

#if 0
  testTx();
#endif

  adcLogging();

	return 0;
}
