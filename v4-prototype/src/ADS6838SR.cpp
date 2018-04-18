#include "ADS6838SR.h"

//define static member variables definitions
uint8_t ads6838::transfer_state;
uint8_t ads6838::select_state;
uint8_t ads6838::tx_buffer_read8[16];

/**
	Converts from uint8_t to bitset
	@param a byte in uint8_t format
	@return a byte in std::bitset<8> format
**/
std::bitset<8> to_bits(uint8_t byte)
{
    return std::bitset<8>(byte);
}


/**
	Initialize ads6838 internal class variables
		https://docs.particle.io/reference/firmware/photon/#setbitorder-
		https://community.particle.io/t/hardware-spi-setup/20093/7
	Place
	 	ads6838 my_class_instance;
	outside the setup() function
	@param null
	@return null
**/
ads6838::ads6838(){
	uint8_t commandByte;
	uint8_t k = 0;
	for (uint8_t i = 0; i < sizeof(this->tx_buffer_read8); i++) {
		if(i%2 == 0) {
			commandByte = (0x04 << 1) & 0xfe; //[15:9] reg addr, [8] read/write
		} else {
			// [7] always 0, [6:4] channel sel, [3:1] range sel, [0] temp sel
			commandByte = k; k++;
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

/**
	Initialize the SPI interface for Particle photon
	Place function in setup()
	@param uint8_t clk_speed , desired clock speed in MHz
	@return null
**/
void ads6838::init(uint8_t clk_speed){
	// Must call init() once, pass optional clk_speed argument to specify
	// a clk other than default (20 MHZ) as an unsigned 8 bit int
	pinMode(this->_SS, OUTPUT);
	digitalWrite(this->_SS, HIGH);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockSpeed(clk_speed, MHZ);
	// https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
	SPI.setDataMode(SPI_MODE1);
	SPI.begin();
}

/**
	Reads all 8 channels of ads6838 asynchronously and sets the passed flag
	variable when complete
	@param uint8_t flag, set to 1 when read is complete
	@return null
**/
void ads6838::read8_DMA(uint8_t &flag){
	std::memcpy(this->tx_buffer, this->tx_buffer_read8, sizeof(this->tx_buffer_read8));
	transfer_state = 0x00;
	SPI.transfer(tx_buffer, rx_buffer, sizeof(rx_buffer), ads6838::trasnferHandler);
	flag = transfer_state;
}


void ads6838::trasnferHandler(){
	transfer_state = 1;
}


/**
	copies result of read8_DMA into passed rx_dest array
	@param uint8_t* rx_dest, must have equal size to ads6838->rx_buffer
	@return null
**/
void ads6838::get8_DMA(uint8_t* rx_dest){
	this->transfer_state = 0;
	// TODO: Add postprocessing to turn get the actual values out of the rx buffer
	std::memcpy(rx_dest, this->rx_buffer, sizeof(this->rx_buffer));
}

/**
	TEST FUNCTION
	@param uint8_t flag, set to 1 when read is complete
	@return null
**/
void ads6838::read8(){
	uint8_t commandByte;
  uint8_t result[2];
	for(uint8_t x=0;x<8;x++) {
    // set to adc#
		// 000 = Ranges as selected through the configuration registers (address 10h to 13h, page 0)
		// 001 = Range is set to ±10V
		// 010 = Range is set to ±5V
		// 011 = Range is set to ±2.5V
		// 100 = Reserved; do not use this setting
		// 101 = Range is set to 0V to 10V
		// 110 = Range is set to 0V to 5V
		// 111 = Powers down the device immediately after the 16th SCLK falling edge
		commandByte = 1; // x; //FIXME
		commandByte = commandByte << 4;
		commandByte = commandByte & 0x70;
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
		delayMicroseconds(1);
    result[0] = SPI.transfer(0);
    result[1] = SPI.transfer(0);

    // disable ADC SPI slave select
    digitalWrite(this->_SS, HIGH);

		std::bitset<8> r1 = to_bits(result[0]);
		std::bitset<8> r2 = to_bits(result[1]);

		Serial.print("AD");
    Serial.print(x);
		Serial.print(" = ");
		for (size_t i = 0; i < 8; i++) {
			Serial.print(r1[i]);
		}
		for (size_t i = 0; i < 8; i++) {
			Serial.print(r2[i]);
		}
		Serial.print(" ");
    Serial.println();
    delay(100);
  }
}
