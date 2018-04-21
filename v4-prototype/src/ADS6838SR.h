#ifndef __ADS6838_INCLUDED__
#define __ADS6838_INCLUDED__

#include "Particle.h"
#include <bitset>

#define ADS8638_REG_MANUAL         0x04
#define ADS8638_REG_AUTO           0x05
#define ADS8638_REG_RESET          0x01
#define ADS8638_REG_AUX_CONFIG     0x06
#define ADS8638_REG_CHAN_SEL       0x0C // channel select for auto mode
#define ADS8638_REG_RANGE_SEL_BASE 0x10

#define ADS8638_REG_TEMP_FLAG      0x20
#define ADS8638_REG_TRIPPED_FLAG_0 0x21
#define ADS8638_REG_ALARM_FLAG_0   0x22
#define ADS8638_REG_TRIPPED_FLAG_4 0x23
#define ADS8638_REG_ALARM_FLAG_4   0x24

#define ADS8638_REG_PAGE_SEL       0x7F

#define ADS8638_READ_FLAG 0x1

#define ADS8638_RANGE_CONFIG   0x0 // Ranges as selected through the configuration registers
#define ADS8638_RANGE_10V      0x1 // Range is set to ±10V
#define ADS8638_RANGE_5V       0x2 // Range is set to ±5V
#define ADS8638_RANGE_2_5V     0x3 // Range is set to ±2.5V
#define ADS8638_RANGE_PLUS_10V 0x5 // Range is set to 0V to 10V
#define ADS8638_RANGE_PLUS_5V  0x6 // Range is set to 0V to 5V
#define ADS8638_POWER_DOWN     0x7 // Powers down the device immediately
                                   // after the 16th SCLK falling edge

#define ADS8638_INTERNAL_VREF_ON 0xC

#define ADS8638_SPI_WRITE_FLAG 0x1

// SPI pin definitions
// #define _SS A2;
// #define _SCLK A3;
// #define _SDOUT A4;
// #define _SDIN A5;
class ads6838
{
public:
	ads6838(); //passing initialized SPI object, desired clk
	void get8_DMA(uint8_t* rx_dest);
	void read8_DMA(uint8_t &flag);
	void read8();
	void init(uint8_t clk_speed = 20);

protected:
	// bit addresses of analog pins
	// static const std::bitset<4> addr[9] = {
	// 	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8
	// };
	// static const enum addr_index = {
	// 	volt1, curr1, curr2, curr3, curr4, curr5, curr6, volt2, temp
	// };
	// SPI pin definitions
	const uint8_t _SSA0 = A1;
	const uint8_t _SSA1 = A0;
	const uint8_t _SS = A2;
	const uint8_t _SCLK = A3;
	const uint8_t _SDOUT = A4;
	const uint8_t _SDIN = A5;

	// buffers for 128 bit SPI transfer, i.e. 16 bytes
	static uint8_t rx_buffer[16];
	static uint8_t tx_buffer[16];
	static uint8_t tx_buffer_read8[16]; //reference for read8 SPI command
	static uint8_t select_state;
	static uint8_t transfer_state;

	static void trasnferHandler();
	void set_tx();
	void transfer128(uint16_t &flag);
};

#endif
