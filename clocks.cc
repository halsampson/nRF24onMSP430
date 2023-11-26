#include <clocks.h>
#include <msp430.h>


// 24 MHz / 32 * N / 32 FLL steps   DCO max 135 MHz -> FLL Div ~180 max

#define REFO_Hz  32768L

void SetVCoreUp (unsigned int level) { // Note: Change core voltage one level at a time.
	PMMCTL0_H = 0xA5; // Open PMM registers for write access

	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level; // Set SVS/SVM high side new level
	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level; // Set SVM low side to new level

	while ((PMMIFG & SVSMLDLYIFG) == 0);// Wait till SVM is settled

	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);// Clear already set flags
	PMMCTL0_L = PMMCOREV0 * level;// Set VCore to new level

	if ((PMMIFG & SVMLIFG))
	  while ((PMMIFG & SVMLVLRIFG) == 0); // Wait till new level reached

	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level; // Set SVS/SVM low side to new level

	while ((PMMIFG & SVSMHDLYIFG) == 0);
	while ((PMMIFG & SVSMLDLYIFG) == 0);

	PMMCTL0_H = 0x00; // Lock PMM registers for write access
}

void stableDCO() {
  while (1) {
  	unsigned int stable = 1024;
  	int lastUCSCTL0 = UCSCTL0;
  	while (abs((int)UCSCTL0 - lastUCSCTL0) <= 8 * 4) // ~ 0.25% MOD taps; stable within 1 %
  		if (!stable--) return;
  }
}

long actualCPUHz;

long setCPUClockREFO(long CPUHz) {
	for (int level = 1; level <= 3; level++) // maximize Vcore for fast MCLK
		SetVCoreUp(level);

	int FLLn;
  int FLLPow = 0;
	while (1) {
	  FLLn = ((CPUHz >> FLLPow) + REFO_Hz / 2) / REFO_Hz - 1;   // FLLN - 10 bits
	  if (FLLn <= 1023) break;
	  ++FLLPow;
	}

  while (UCA0STAT & UCBUSY);
	UCA0CTL1 = UCSWRST;

	UCSCTL4 = SELA_3 | SELS_2 | SELM_2;        // Select MCLK = REFOCLK while stabilizing DCO;  ACLK from DCO to enable it

  __bis_SR_register(SCG0);                 // Disable the FLL control loop
	UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFOCLK

  // could auto-adjust by watching UCSCTLO for DCO = 0 -> down  or 31 -> up
	UCSCTL1 = (CPUHz << FLLPow) >= 28000000L ? DCORSEL_7 : DCORSEL_6;    // fDCO(n, 0),MAX ≤ fDCO ≤ fDCO(n, 31),MIN   6: 10.7 to 39.0 MHz   7: 19.6 to 60 MHz for BT5190 p. 25  / 2

  UCSCTL2 = FLLPow * FLLD_1 | FLLn;
  UCSCTL7 &= ~DCOFFG;
	__bic_SR_register(SCG0);                // Enable the FLL control loop

	stableDCO();

	UCSCTL4 = SELA_3 | SELS_3 | SELM_3;         // all from  DCOCLK; or any / 2 with DCOCLKDIV

	// UCA0BRW = (CPUHz + BaudRate / 2) / BaudRate;
	UCA0CTL1 = UCSSEL_1; // ACLK

	UCSCTL7 &= ~DCOFFG;
	SFRIFG1 &= ~OFIFG;

	return actualCPUHz = (FLLn + 1) * REFO_Hz << FLLPow;
}


void delay_us(word us) {
	// static byte us_cycles = actualCPUHz / 1000000 - 6;
	const byte us_cycles = NomCPUHz / 1000000 - 6; // TODO: adjust loop overhead, beware CPUHz < 6 MHz ****
	while (us--)
		__delay_cycles(us_cycles);
}


void delay(word ms) {  // TODO: better sleep with timer wake: RTC 32kHz
	// static word ms_cycles = actualCPUHz / 1000 - 6;  // adjust for while loop
	const word ms_cycles = NomCPUHz / 1000 - 6;
	while (ms--)
		__delay_cycles(ms_cycles);
}

