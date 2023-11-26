#ifndef NRF24LITE_H_
#define NRF24LITE_H_

#include <nRF24L01.h>
#include <SPIlite.h>

// configure IRQ
#define nRF24IRQ     ((UCAx*)USCI_A3_BASE)
#define P10_BASE_OK  (P10_BASE + 1)          // P10_BASE == P9_BASE - wrong!!
#define nRF24IRQport ((PortB*)P10_BASE_OK)
#define nRF24IRQpin  BIT5 // UCA3RxD on JP12 wire to JP1-8 IRQ

const byte PayloadSize = 32; // max if dynamic
extern byte status;

void write_register(byte reg, byte b);
void write_register(byte reg, const void* b, byte len);

byte read_register(byte reg);
void read_register(byte reg, void* buf, byte len);

byte get_status();

void flush_rx(void);
void flush_tx(void);

void initRF24();

extern byte usedChannels[128 / 8];
byte scanChannels();
void setChannel(byte channel);

void setPAlevel(byte level); // 0..3: * 6 - 18 = dBm out  lower for YJ-25008+PA? vs. overload, esp 3.6V
byte getPAlevel();

void openReadingPipe(byte pipe, const void* address);
void closeReadingPipe(byte pipe);

void startListening(void);
void stopListening(void);

void openWritingPipe(const void* address);
bool write(const void* buf, int8 len = -1);   // defaults to strlen


#endif /* NRF24LITE_H_ */
