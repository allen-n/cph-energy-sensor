#include "ADS6838SR.h"

#include <bitset>
//define static member variables
uint8_t ads6838::transfer_state;
uint8_t ads6838::select_state;
uint8_t ads6838::tx_buffer_read8[16];

ads6838::ads6838(){
	/**
		Initialize ads6838 for SPI, pass Particle.h SPI object as SPI
		see:
			https://docs.particle.io/reference/firmware/photon/#setbitorder-
			https://community.particle.io/t/hardware-spi-setup/20093/7
		Place ads6838() in void setup()
	**/
	// for (int i = 0; i < sizeof(tx_buffer); i++){
	// 	tx_buffer[i] = (uint8_t)i;
	// }
	uint8_t commandByte;
	for (uint8_t i = 0; i < sizeof(this->tx_buffer_read8); i++) {
		if(i%2 == 0) {
			commandByte = 0x04;
		} else {
			commandByte = i;
			commandByte = commandByte << 4;
			commandByte = commandByte & 0x70;
			uint8_t range = (0x03 << 1) & 0x0e;
			commandByte = commandByte | range;
		}
		this->tx_buffer_read8[i] = commandByte;
	}
	this->transfer_state = 0x00;
	this->select_state = 0x00;
}

void ads6838::init(uint8_t clk_speed){
		pinMode(this->_SS, OUTPUT); //maybe unnecssasary, begin() does this
		digitalWrite(this->_SS, HIGH);
		SPI.setBitOrder(MSBFIRST);
		SPI.setClockSpeed(clk_speed, MHZ);
		// https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
		SPI.setDataMode(SPI_MODE1);
		SPI.begin();
}

// read a uint8_t of data from the ads6838
// pass register to be read in val
// value of register is returned in val


void ads6838::read8_DMA(uint8_t &flag){
	std::memcpy(this->tx_buffer, this->tx_buffer_read8, sizeof(this->tx_buffer_read8));
	transfer_state = 0x00;
	SPI.transfer(tx_buffer, rx_buffer, sizeof(rx_buffer), ads6838::trasnferHandler);
	flag = transfer_state;
}

void ads6838::trasnferHandler(){
	transfer_state = 1;
}

void ads6838::get8_DMA(uint8_t* rx_dest){
	this->transfer_state = 0;
	// TODO: Add postprocessing to turn get the actual values out of the rx buffer
	std::memcpy(rx_dest, this->rx_buffer, sizeof(this->rx_buffer));
}

void ads6838::read8(){
	uint8_t commandByte;
  uint8_t result[2];
	for(uint8_t x=0;x<8;x++) {
    // set to adc#		
    commandByte = 1; // x;
    // shift into position and set bit 7 to LOW -- 1011000
		// want: 00000100
		// want: 00010110
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
		uint8_t range = (0x03 << 1) & 0x0e;
		commandByte = commandByte | range;
		// commandByte = commandByte | 0x01; //get temp values, FIXME
		//zero last bit to prevent temp value
		// commandByte = commandByte & 0xfe;

    // enable ADC SPI slave select
    digitalWrite(this->_SS, LOW);
    delayMicroseconds(1);
		SPI.transfer(0x04); //manual register addressing command
    SPI.transfer(commandByte);
    // you might need to increase this delay for conversion
		digitalWrite(this->_SS, HIGH);
    delayMicroseconds(5);
		// while(!(SPI.available())); //better alternative to blocking code
    // get results
		digitalWrite(this->_SS, LOW);
    result[0] = SPI.transfer(0);
    result[1] = SPI.transfer(0);

    // disable ADC SPI slave select
    digitalWrite(this->_SS, HIGH);

		// std::bitset<>
    Serial.print("AD");
    Serial.print(x);
    Serial.print(" = ");
    Serial.print(result[0]);
    Serial.print(" ");
    Serial.println(result[1]);
    delay(100);
  }
}
