/**
 * @file custom_chars.h
 * @brief Defines custom character bitmaps for the LCD display.
 * @author kiril
 * @date Jul 26, 2025
 */

#ifndef INC_CUSTOM_CHARS_H_
#define INC_CUSTOM_CHARS_H_

#include <inttypes.h>

/** @brief Custom character for Temperature symbol (thermometer). */
extern const uint8_t symbol_temp[8];
/** @brief Custom character for Warning symbol (exclamation mark). */
extern const uint8_t symbol_warning[8];
/** @brief Custom character for Danger/Shutdown symbol (skull). */
extern const uint8_t symbol_danger[8];
/** @brief Custom character for 'On' state (play icon). */
extern const uint8_t symbol_on[8];
/** @brief Custom character for 'Off' state (pause icon). */
extern const uint8_t symbol_off[8];

#endif /* INC_CUSTOM_CHARS_H_ */
