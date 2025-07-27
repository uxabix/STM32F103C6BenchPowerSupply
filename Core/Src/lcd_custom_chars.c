/*
 * lcd_custom_chars.c
 *
 *  Created on: Jul 26, 2025
 *      Author: kiril
 */

#include "lcd_custom_chars.h"


const uint8_t symbol_temp[8] = {
	0b11100,
	0b10100,
	0b11100,
	0b00011,
	0b00100,
	0b00100,
	0b00100,
	0b00011
};
const uint8_t symbol_warning[8] = {
	0b00100,
	0b01110,
	0b01110,
	0b01110,
	0b00100,
	0b00000,
	0b00100,
	0b00000
};
const uint8_t symbol_danger[8] = {
	0b00000,
	0b10001,
	0b01010,
	0b00100,
	0b01010,
	0b10001,
	0b00000,
	0b11111
};
const uint8_t symbol_on[8] = {
	0b01000,
	0b01100,
	0b01110,
	0b01111,
	0b01110,
	0b01100,
	0b01000,
	0b00000
};
const uint8_t symbol_off[8] = {
	0b00000,
	0b01010,
	0b01010,
	0b01010,
	0b01010,
	0b01010,
	0b01010,
	0b00000
};
