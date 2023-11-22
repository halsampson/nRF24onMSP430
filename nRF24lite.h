#ifndef NRF24LITE_H_
#define NRF24LITE_H_

#include <nRF24L01.h>
#include <SPIlite.h>

extern byte status;

void write_register(byte reg, byte b);
void write_register(byte reg, const byte* b, byte len);

byte read_register(byte reg);
void read_register(byte reg, byte* buf, byte len);

byte get_status();

void flush_rx(void);
void flush_tx(void);

void initRF24();

void openReadingPipe(byte pipe, const byte* address);
void closeReadingPipe(byte pipe);

void startListening(void);
void stopListening(void);

void openWritingPipe(const byte* address);
bool write(const byte* buf, int8 len = -1);


#endif /* NRF24LITE_H_ */
