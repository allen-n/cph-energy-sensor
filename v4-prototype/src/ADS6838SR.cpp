#include "ADS6838SR.h"

ads6838::ads6838(uint8_t clk_speed = 20){
	/**
		Initialize ads6838 for SPI, pass Particle.h SPI object as SPI
		see:
			https://docs.particle.io/reference/firmware/photon/#setbitorder-
			https://community.particle.io/t/hardware-spi-setup/20093/7
		Place ads6838() in void setup()
	**/
	pinMode(this->_SS, OUTPUT); //maybe unnecssasary, begin() does this
	digitalWrite(this->_SS, HIGH);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockSpeed(clk_speed, MHZ);
	// https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
	SPI.setDataMode(SPI_MODE1);
	for (int i = 0; i < sizeof(tx_buffer); i++){
		tx_buffer[i] = (uint8_t)i;
	}
	transfer_state = 0x00;
	select_state = 0x00;
	SPI.begin();
}

// read a byte of data from the ads6838
// pass register to be read in val
// value of register is returned in val
void ads6838::read8(){
	byte commandByte;
  byte result[2];
	for(byte x=0;x<8;x++) {
    // set to adc#
    commandByte = x;
    // shift into position and set bit 7 to LOW
    commandByte = commandByte << 4;
		commandByte = commandByte & 0x70;
    // set bits 3:1  for range:
		// 000 = Ranges as selected through the configuration registers (address 10h to 13h, page 0)
		// 001 = Range is set to ±10V
		// 010 = Range is set to ±5V
		// 011 = Range is set to ±2.5V
		// 100 = Reserved; do not use this setting
		// 101 = Range is set to 0V to 10V
		// 110 = Range is set to 0V to 5V
		// 111 = Powers down the device immediately after the 16th SCLK falling edge
		byte range = (0x03 << 1) & 0x0e;
		commandByte = commandByte | range;
		//zero last bit to prevent temp value
		// commandByte = commandByte & 0xfe;

    // enable ADC SPI slave select
    digitalWrite(this->_SS, LOW);
    delayMicroseconds(1);

    SPI.transfer(commandByte);
    // you might need to increase this delay for conversion
    delayMicroseconds(2);
		// while(!(SPI.available())); //better alternative to blocking code

    // get results
    result[0] = SPI.transfer(0);
    result[1] = SPI.transfer(0);

    // disable ADC SPI slave select
    digitalWrite(this->_SS, HIGH);

    Serial.print("AD");
    Serial.print(x);
    Serial.print(" = ");
    Serial.print(result[0]);
    Serial.print(" ");
    Serial.println(result[1]);
    delay(100);
  }
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
