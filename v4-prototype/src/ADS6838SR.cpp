#include "ADS6838SR.h"

// //define static member variables definitions
// uint8_t ads6838::transfer_state;
// uint8_t ads6838::select_state;
// uint8_t ads6838::tx_buffer_read8[16];

// // NOTE: For error logging and seeing actual bit values
// /**
// 	Converts from uint8_t to bitset
// 	@param byte ;a byte in uint8_t format
// 	@return a byte in std::bitset<8> format
// **/
// std::bitset<8> to_bits(uint8_t byte)
// {
//     return std::bitset<8>(byte);
// }
//
// std::bitset<16> to_bits(uint8_t msb, uint8_t lsb)
// {
//     uint16_t byte = msb;
//     byte =  (byte << 8);
//     byte = byte | lsb;
//     return std::bitset<16>(byte);
// }

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
	// uint8_t commandByte;
	// uint8_t k = 0;
  this->_range = ADS8638_RANGE_5V; //FIXME, this range works, new one may not
  // this->_range = ADS8638_RANGE_2_5V;
	// for (uint8_t i = 0; i < sizeof(this->tx_buffer_read8); i++) {
	// 	if(i%2 == 0) {
	// 		commandByte = (ADS8638_REG_MANUAL << 1) & 0xfe; //[15:9] reg addr, [8] read/write
	// 	} else {
	// 		// [7] always 0, [6:4] channel sel, [3:1] range sel, [0] temp sel
	// 		commandByte = k; k++;
	// 		commandByte = commandByte << 4;
	// 		commandByte = commandByte & 0x70;
	// 		uint8_t range = (ADS8638_RANGE_2_5V << 1) & 0x0e;
	// 		commandByte = commandByte | range;
	// 	}
	// 	this->tx_buffer_read8[i] = commandByte;
	// }
	// this->transfer_state = 0x00;
	// this->select_state = 0x00;
}

/**
	Initialize the SPI interface for Particle photon
	Place function in setup()
	@param uint8_t clk_speed , desired clock speed in MHz
	@return null
**/

void ads6838::writeCmd(uint8_t addr, uint8_t cmd){
  digitalWrite(this->_SS, LOW);
  // delayMicroseconds(1);
  SPI.transfer((addr << 1) & 0xfe); //manual register addressing command
  SPI.transfer(cmd);
  // you might need to increase this delay for conversion
  digitalWrite(this->_SS, HIGH);
}

uint8_t ads6838::readReg(uint8_t addr){
  uint8_t out;
  digitalWrite(this->_SS, LOW);
  // delayMicroseconds(1);
  SPI.transfer((addr << 1) | 0x01); //manual register addressing command
  out = SPI.transfer(0);
  // you might need to increase this delay for conversion
  digitalWrite(this->_SS, HIGH);

  return out;
}


void ads6838::init(uint8_t clk_speed, uint8_t range){
	// Must call init() once, pass optional clk_speed argument to specify
	// a clk other than default (20 MHZ) as an unsigned 8 bit int
	pinMode(this->_SS, OUTPUT);
  pinMode(this->_SSA0, OUTPUT);
  pinMode(this->_SSA1, OUTPUT);
	digitalWrite(this->_SS, HIGH);
  // selecting correct SPI interface on digital isolator
  digitalWrite(this->_SSA0, LOW);
  digitalWrite(this->_SSA1, LOW);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockSpeed(clk_speed, MHZ);
	// https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
	SPI.setDataMode(SPI_MODE1);
	SPI.begin();
  this->_range = range;
  // uint8_t range = this->_range;
  range = (range << 4) | range;
  writeCmd(0x10, range);
  writeCmd(0x11, range);
  writeCmd(0x12, range);
  writeCmd(0x13, range);
  writeCmd(0x06, 0x04); //configure internal reference
  SPI.transfer(0);
  SPI.transfer(0);
}

// /**
// 	Reads all 8 channels of ads6838 asynchronously and sets the passed flag
// 	variable when complete
// 	@param uint8_t flag, set to 1 when read is complete
// 	@return null
// **/
// void ads6838::read8_DMA(uint8_t &flag){
// 	std::memcpy(this->tx_buffer, this->tx_buffer_read8, sizeof(this->tx_buffer_read8));
// 	transfer_state = 0x00;
// 	SPI.transfer(tx_buffer, rx_buffer, sizeof(rx_buffer), ads6838::trasnferHandler);
// 	flag = transfer_state;
// }
//
//
// void ads6838::trasnferHandler(){
// 	transfer_state = 1;
// }


// /**
// 	copies result of read8_DMA into passed rx_dest array
// 	@param uint8_t* rx_dest, must have equal size to ads6838->rx_buffer
// 	@return null
// **/
// void ads6838::get8_DMA(uint8_t* rx_dest){
// 	this->transfer_state = 0;
// 	// TODO: Add postprocessing to turn get the actual values out of the rx buffer
// 	std::memcpy(rx_dest, this->rx_buffer, sizeof(this->rx_buffer));
// }

void ads6838::selectChannel(uint8_t channel){
  digitalWrite(this->_SS, LOW);
  // delayMicroseconds(1);
  SPI.transfer(ADS8638_REG_MANUAL << 1); //manual register addressing command
  SPI.transfer((channel << 4) | (this->_range << 1)); //manual register addressing command

  // you might need to increase this delay for conversion
  digitalWrite(this->_SS, HIGH);
}

uint16_t ads6838::read1(uint8_t channel, uint8_t range){
	uint8_t commandByte;
  uint8_t result[2];
	commandByte = (channel << 4) | (range << 1);

  digitalWrite(this->_SS, LOW);
  // delayMicroseconds(1);
	SPI.transfer(ADS8638_REG_MANUAL << 1); //manual register addressing command
  SPI.transfer(commandByte);
  // you might need to increase this delay for conversion
	digitalWrite(this->_SS, HIGH);


  // delayMicroseconds(1);
  // get results
	digitalWrite(this->_SS, LOW);
	// delayMicroseconds(1);
	// while(!(SPI.available())); // better alternative to blocking code, may not work
  SPI.transfer(0x00);
  SPI.transfer(0x00);

	// delayMicroseconds(1);
  // disable ADC SPI slave select
  digitalWrite(this->_SS, HIGH);

  digitalWrite(this->_SS, LOW);
	// delayMicroseconds(1);
	// while(!(SPI.available())); // better alternative to blocking code, may not work
  result[0] = SPI.transfer(0x00);
  result[1] = SPI.transfer(0x00);

	// delayMicroseconds(1);
  // disable ADC SPI slave select
  digitalWrite(this->_SS, HIGH);

  uint16_t out = result[0];
  out = (out << 8) | result[1];
  out = out & 0x0fff; //NOTE: For printing response
  // out = (out & 0xf000) >> 12; //NOTE: For printing address of resp
  return out;


}

/**
	TEST FUNCTION
	@param uint8_t flag, set to 1 when read is complete
	@return null
**/
void ads6838::read8(uint16_t* out, uint8_t addr, uint8_t range){
	uint8_t commandByte;
  uint8_t result[2];
  uint8_t commandAddr = (addr << 1); // & 0xfe;

	for(uint8_t x=0;x<8;x++) { // FIXME: just checking probed curred1 track

		commandByte = (x << 4) | (range << 1);
		// commandByte = commandByte | 0x01; //get temp values, FIXME

    // // NOTE: DEBUG for command word
    // std::bitset<8> addr_byte = to_bits((addr << 1));
    // std::bitset<8> cmd_byte = to_bits(commandByte);
    // Serial.print("Command Byte: ");
    // for (int i = 7; i >= 0; --i) {
		// 	Serial.print(addr_byte[i]);
		// }
    // Serial.print(" ");
    // for (int i = 7; i >= 0; --i) {
		// 	Serial.print(cmd_byte[i]);
		// }
    // Serial.println("");
    // chip select on
    // enable ADC SPI slave select
    digitalWrite(this->_SS, LOW);
    // delayMicroseconds(1);
		SPI.transfer(commandAddr); //manual register addressing command
    SPI.transfer(commandByte);
    // you might need to increase this delay for conversion
		digitalWrite(this->_SS, HIGH);


    // delayMicroseconds(1);
    // get results
		digitalWrite(this->_SS, LOW);
		// delayMicroseconds(1);
		// while(!(SPI.available())); // better alternative to blocking code, may not work
    result[0] = SPI.transfer(0x00);
    result[1] = SPI.transfer(0x00);

		// delayMicroseconds(1);
    // disable ADC SPI slave select
    digitalWrite(this->_SS, HIGH);

    out[x] = result[0];
    out[x] = (out[x] << 8) | result[1];
    out[x] = out[x] & 0x0fff; //NOTE: For printing response
    // out[x] = (out[x] & 0xf000) >> 12; //NOTE: For printing address of resp
    // Serial.println(out[1]);
    // uint8_t reg = (result[0] >> 4) & 0x0f;
    // uint16_t val = (result[0]) & 0x0f;
    // val = (val << 8) | result[1];
    //
    // // // NOTE: Debug for SPI output
		// Serial.print("AD");
    // Serial.print(x);
		// Serial.print(" = ");
    // Serial.print(reg);Serial.print(" : ");
    // Serial.print(val);Serial.print(" : ");
    // std::bitset<16> res = to_bits(result[0], result[1]);
    // for (int i = 15; i >= 0; --i) {
		// 	Serial.print(res[i]);
		// }
		// Serial.print(" ");
    // Serial.println();
    // delay(100);
  }
}
