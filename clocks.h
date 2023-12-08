#ifndef CLOCKS_H_
#define CLOCKS_H_

typedef unsigned char byte;
typedef   signed char int8;
typedef unsigned int  word;
typedef unsigned long uint32;
typedef unsigned long long uint64;

// 24 MHz / 32 * N / 32 FLL steps   DCO max 135 MHz -> FLL Div ~180 max
const uint32 BaudRate = 921600L;
const uint32 NomCPUHz = 8 * BaudRate;

const word REFOCLK_HZ = 32768;

long setCPUClockREFO(long CPUHz);
extern long actualCPUHz;

void delay_us(word us);
void delay(word ms);


#endif /* CLOCKS_H_ */
