/**
 * @file custom_chars.c
 * @brief Defines the bitmaps for custom characters displayed on the LCD.
 * @author kiril
 * @date Jul 26, 2025
 *
 * @details Each array represents a 5x8 pixel custom character. The LCD can store
 *          up to 8 custom characters in its CGRAM.
 */

#include "custom_chars.h"

/** @brief Custom character for Temperature symbol (thermometer). */
uint8_t symbol_temp[8] = {
	0b11100,
	0b10100,	0b11100,
	0b00011,
	0b00100,
	0b00100,
	0b00100,
	0b00011
};

/** @brief Custom character for Warning symbol (exclamation mark). */
uint8_t symbol_warning[8] = {
	0b00000,
	0b01110,
	0b01110,
	0b01110,
	0b00100,
	0b00000,
	0b00100,
	0b00000
};
/** @brief Custom character for Danger/Shutdown symbol. */
uint8_t symbol_danger[8] = { // A skull and crossbones icon
	0b00000,
	0b10001,
	0b01010,
	0b00100,
	0b01010,
	0b10001,
	0b00000,
	0b11111
};
/** @brief Custom character for 'On' state (play icon / filled triangle). */
uint8_t symbol_on[8] = {
	0b01000,
	0b01100,
	0b01110,
	0b01111,
	0b01110,
	0b01100,
	0b01000,
	0b00000
};
/** @brief Custom character for 'Off' state (pause icon / two vertical bars). */
uint8_t symbol_off[8] = {
	0b00000,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b00000
};
