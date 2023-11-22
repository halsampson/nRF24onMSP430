#include <nRF24lite.h>
#include <SPIlite.h>

byte status;

void write_register(byte reg, byte b) {
	status = xferSPI(W_REGISTER | reg);
	xferSPI(b);

	nRF24port->Out |= CSN;
}

void write_register(byte reg, const byte* b, byte len) {
	status = xferSPI(W_REGISTER | reg);
	while (len--)
	  xferSPI(*b++);

	nRF24port->Out |= CSN;
}


byte read_register(byte reg) {
	status = xferSPI(R_REGISTER | reg);
	byte got = xferSPI(0xff);

	nRF24port->Out |= CSN;
	return got;
}

void read_register(byte reg, byte* buf, byte len) {
	status = xferSPI(R_REGISTER | reg);
	while (len--)
		*buf++ = xferSPI(RF24_NOP);
	nRF24port->Out |= CSN;
}


byte get_status() {
  write_register(RF24_NOP, RF24_NOP);
  return status;
}

void flush_rx(void) {
    write_register(FLUSH_RX, RF24_NOP);
}

void flush_tx(void) {
    write_register(FLUSH_TX, RF24_NOP);
}


const byte PayloadSize = 8; // or can be dynamic

void initRF24() {
	nRF24port->Out &= ~CE;
	nRF24port->Out |= CSN;
	delay(5);

  write_register(EN_AA, 0x3F);       // enable auto-ack on all pipes
  write_register(EN_RXADDR, 3);      // only open RX pipes 0 & 1
  write_register(SETUP_AW, 5 - 2);   // set address length to 5 bytes
  write_register(SETUP_RETR, 15 << 4 | 15); // (n * 250us + 1) interval between m retransmits
  write_register(RF_CH, 1);
	write_register(RF_SETUP, 1 << RF_DR_LOW | 3 << 1 | 1); // 250 kbps | max power | lnaEnable

	for (byte pipe = 0; pipe <= 5; ++pipe) {
    write_register(RX_PW_P0 + pipe, PayloadSize);  // set static payload size to (max) bytes
    write_register(RX_ADDR_P0 + pipe, 0xc1 + pipe);  // unique addresses
	}

	write_register(DYNPD, 0);     // disable dynamic payload size

	write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); //reset
	flush_rx();
	flush_tx();

	write_register(NRF_CONFIG, 1 << EN_CRC | 1 << CRCO | 1 << PWR_UP); // CRC16, Power, Tx mode
}


void openReadingPipe(byte pipe, const byte* address) {  // LSB first
  write_register(RX_ADDR_P0 + pipe, address, pipe <= 1 ? 5 : 1);
  write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << pipe);
}

void closeReadingPipe(byte pipe) {
  write_register(EN_RXADDR, read_register(EN_RXADDR) & ~(1 << pipe));
}

void startListening(void) {
  write_register(NRF_CONFIG, 1 << EN_CRC | 1 << CRCO | 1 << PWR_UP | 1 << PRIM_RX);
  write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); // reset
  nRF24port->Out |= CE;
}

void stopListening(void){
  nRF24port->Out &= ~ CE;

  delay_us(100);
  flush_tx();  // any ACKs

  write_register(NRF_CONFIG,  1 << EN_CRC | 1 << CRCO | 1 << PWR_UP);
  write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << 0); // Enable RX on pipe0
}


void openWritingPipe(const byte* address) {  // LSB first
  write_register(RX_ADDR_P0, address, 5);  // for ACKs
  write_register(TX_ADDR, address, 5);
}


bool write(const byte* buf, int8 len /* = -1*/) {  // defaults to null-terminated if no len given
	xferSPI(W_TX_PAYLOAD);  // w/ ACK

	byte payloadLen = PayloadSize; // better variable
	while (payloadLen-- && len-- && (len >= 0 || *buf))
	  xferSPI(*buf++);

	while (payloadLen--)  // pad to PayloadLen
		xferSPI(0);

	nRF24port->Out |= CSN | CE; // Tx active high pulse > 10 us to send payload

	bool OK;
	byte timeout = 120; // ms  (1 + 5 + 1 + 32 + 2 + 1) = 42 bytes / 250 kHz = (1.344ms * (tx + ack ) + retry (4ms)) * 15 retries
  while (1) {
  	if (get_status() & (1 << TX_DS)) { // DataSent
  		OK = true;
  		break;
  	}

    if (status & (1 << MAX_RT) || !--timeout) { // Max retries exceeded or timeout
      OK = false;
      flush_tx(); // only 1 packet in FIFO, so just flush
      break;
    }
    delay(1);  // or less
  }

	nRF24port->Out &= ~CE;
	write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); // reset status bits
	return OK;
}
