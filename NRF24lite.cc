#include <nRF24lite.h>
#include <SPIlite.h>

byte status;

byte read_register(byte reg) {
	status = xferSPI(R_REGISTER | reg);
	byte got = xferSPI(0xff);
	nRF24port->Out |= CSN;
	return got;
}

void read_register(byte reg, void* buf, byte len) {
	status = xferSPI(R_REGISTER | reg);
	while (len--) {
		*(byte*)buf = xferSPI(RF24_NOP);
		buf = (void*)((byte*)buf + 1);
	}
	nRF24port->Out |= CSN;
}


void write_register(byte reg, byte b) {
	status = xferSPI(W_REGISTER | reg);
	xferSPI(b);
	nRF24port->Out |= CSN;
}

void write_register(byte reg, const void* b, byte len) {
	status = xferSPI(W_REGISTER | reg);
	while (len--) {
	  xferSPI(*(byte*)b);
	  b = (void*)((byte*)b + 1);
	}
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

const byte PacketOvhd = 1 + 1;  // preamble, control
const byte AddressWidth = 5;  // pipe 0 and 1
const byte CRCWidth = 2;
byte packetSize = PacketOvhd + AddressWidth + PayloadSize + CRCWidth;


const byte dataRate = 0 ? 0 : 1 << RF_DR_LOW;  //  1 << RF_DR_HIGH  sets 2 Mbps;  0 = 1 Mbps

void setPAlevel(byte level) { // 0..3: * 6 - 18 = dBm out  lower for YJ-25008+PA? vs. overload, esp 3.6V
	write_register(RF_SETUP, dataRate | (level & 3) << 1);
}

byte getPAlevel() {
	return (read_register(RF_SETUP) >> 1) & 3;
}

void initRF24() {
  initSPI();

	nRF24port->Out &= ~CE;
	nRF24port->Out |= CSN;
	delay(5);

  write_register(EN_AA, 0x3F);       // enable auto-ack on all pipes
  write_register(SETUP_AW, AddressWidth - 2);   // set address length to 5 bytes
  write_register(SETUP_RETR, 15 << 4 | 15); // (n * 250us + 1) interval between m retransmits
  write_register(RF_CH, 1);
  setPAlevel(3);

	for (int8 pipe = 5; pipe >= 0; --pipe)
    write_register(RX_ADDR_P0 + pipe, 0xc1 + pipe);  // unique addresses

	write_register(DYNPD, 0x3F);  // enable dynamic payload size on all pipes
	write_register(FEATURE, 1 << EN_DPL | 1 << EN_ACK_PAY);

	write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); //reset
	flush_rx();
	flush_tx();

	write_register(NRF_CONFIG, 1 << EN_CRC | 1 << CRCO | 1 << PWR_UP); // CRC16, Power, Tx mode
}


void openReadingPipe(byte pipe, const void* address) {  // LSB first
  write_register(RX_ADDR_P0 + pipe, address, pipe <= 1 ? 5 : 1);
  write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << pipe);
}

void closeReadingPipe(byte pipe) {
  write_register(EN_RXADDR, read_register(EN_RXADDR) & ~(1 << pipe));
}

void startListening(void) {
  write_register(NRF_CONFIG, 1 << EN_CRC | 1 << CRCO | 1 << PWR_UP | 1 << PRIM_RX);  // TODO: IRQ MASK
  write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); // reset
  nRF24port->Out |= CE;
}

void stopListening(void){
  nRF24port->Out &= ~CE;

  delay_us(100);
  flush_tx();  // any ACKs

  write_register(NRF_CONFIG,  1 << EN_CRC | 1 << CRCO | 1 << PWR_UP);
  write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << 0); // Enable RX on pipe0
}


void openWritingPipe(const void* address) {  // LSB first
  write_register(TX_ADDR, address, AddressWidth);
  write_register(RX_ADDR_P0, address, AddressWidth);  // for receiving ACK packets
  write_register(EN_RXADDR, read_register(EN_RXADDR) | 3);  // pipes 0 and 1 for TX and RX ACKs

	write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); // reset status bits
	write_register(NRF_CONFIG, 1 << MASK_RX_DR | 1 << EN_CRC | 1 << CRCO | 1 << PWR_UP); // CRC16, Power, Tx mode
  // IRQ pin low on DataSent or MAX_RT retries
}


bool write(const void* buf, int8 len /* = -1*/) {  // defaults to null-terminated if no len given
  // payload width is set by # bytes clocked into TX FIFO

	xferSPI(W_TX_PAYLOAD);  // w/ ACK

	int8 payloadLen = PayloadSize;
	while (len-- && (len >= 0 || *(byte*)buf)) {  // negative len means null terminated string
	  xferSPI(*(byte*)buf);
	  buf = (void*)((byte*)buf + 1);
	  if (--payloadLen <= 0) break;
  }

	if (0) while (payloadLen-- > 0)  // pad to PayloadSize for static
		xferSPI(0);

	write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); // reset status bits

	nRF24port->Out |= CSN | CE; // Tx active high pulse > 10 us to send payload

	// TODO: better sleep and wait for IRQ or timeout timer wake ******

	byte timeout = 120; // ms  (1 + 5 + 1 + 32 + 2 + 1) = 42 bytes / 250 kHz = (1.344ms * (tx + ack ) + retry (4ms)) * 15 retries
  do {
    delay(1); // PLL settle + packetSize * 8 * 4us * (transmit + ack)
  	nRF24port->Out &= ~CE; // or later/earlier (10 us min pulse)
  } while ((nRF24port->IN & IRQ) && --timeout);  // IRQ active LO


  bool OK = get_status() & (1 << TX_DS); // DataSent
  if (!OK) {// Max retries exceeded or timeout
    flush_tx(); // only 1 packet in FIFO, so just flush
    flush_rx();
  }

	write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); // reset status bits
	return OK;
}
