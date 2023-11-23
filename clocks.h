
#ifndef CLOCKS_H_
#define CLOCKS_H_

// 24 MHz / 32 * N / 32 FLL steps   DCO max 135 MHz -> FLL Div ~180 max

const unsigned long BaudRate = 921600L;
const unsigned long NomCPUHz = 16 * BaudRate;

typedef unsigned int word;
typedef unsigned char byte;
typedef signed char int8;

long setCPUClockREFO(long CPUHz);

void delay_us(word us);
void delay(word ms);


#endif /* CLOCKS_H_ */
