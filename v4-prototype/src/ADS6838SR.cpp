#include "ADS6838SR.h"

ads6838::ads6838(){
	/**
		Initialize ads6838 for SPI, pass Particle.h SPI object as SPI
		see:
			https://docs.particle.io/reference/firmware/photon/#setbitorder-
			https://community.particle.io/t/hardware-spi-setup/20093/7
		Place ads6838() in void setup()
	**/
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockSpeed(20, MHZ);
	// https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
	SPI.setDataMode(SPI_MODE1);
	for (int i = 0; i < sizeof(tx_buffer); i++){
		tx_buffer[i] = (uint8_t)i;
	}
	transfer_state = 0x00;
	select_state = 0x00;
}

// read a byte of data from the ads6838
// pass register to be read in val
// value of register is returned in val
void ads6838::read8(uint8_t &val){
	digitalWrite(_SS, LOW);
	SPI.transfer(val);
	val = SPI.transfer(0x00);
	digitalWrite(_SS, HIGH);
}

// read 16?
// uint8_t MSB = (val & 0xF0) >> 8;
// uint8_t LSB = (val & 0x0F);
// digitalWrite(_SS, LOW);
// SPI.transfer(val & 0xF0);
// val = SPI.transfer(val & 0x0F);
// digitalWrite(_SS, HIGH);

// sending all data in tx_buffer, data available in rx_buffer when
// transfer_state flag is = 1
void ads6838::transfer128(uint16_t &flag){
	transfer_state = 0x00;
	SPI.transfer(tx_buffer, rx_buffer, sizeof(rx_buffer), ads6838::trasnferHandler);
	flag = transfer_state;
}

void ads6838::trasnferHandler(){
	transfer_state = 1;
}
