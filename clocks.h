/*
 * clocks.h
 *
 *  Created on: Nov 21, 2023
 *      Author: Admin
 */

#ifndef CLOCKS_H_
#define CLOCKS_H_

// 24 MHz / 32 * N / 32 FLL steps   DCO max 135 MHz -> FLL Div ~180 max

const unsigned long BaudRate  = 921600L;
const unsigned long NomCPUHz = 16 * BaudRate;

long setCPUClockREFO(long CPUHz);

void delay_us(int us);
void delay(int ms);


#endif /* CLOCKS_H_ */
