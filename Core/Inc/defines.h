/**
 * @file defines.h
 * @brief Global definitions and constants for the project.
 * @author kiril
 * @date Jul 7, 2025
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

#include <stdio.h>

/**
 * @def DEBUG
 * @brief Enables or disables debug output.
 *
 * Set to 1 to enable debug print statements (e.g., via `printf()`).
 * Set to 0 to disable debug output for release builds.
 */
#define DEBUG 1  // 1 or 0
#if DEBUG
  #define debug_printf(...) printf(__VA_ARGS__)
#else
  #define debug_printf(...) ((void)0)
#endif

/** @name LCD I2C Configuration */
///@{
#define LCD_ADDR (0x27 << 1)    //!< I2C address of the LCD PCF8574 backpack.
#define PIN_RS    (1 << 0)      //!< PCF8574 pin connected to LCD RS.
#define PIN_EN    (1 << 2)      //!< PCF8574 pin connected to LCD EN.
#define BACKLIGHT (1 << 3)      //!< PCF8574 pin connected to LCD Backlight.
#define LCD_DELAY_MS 1          //!< General delay for LCD commands.
#define SCREEN_LENGTH 16        //!< Width of the LCD screen in characters.
///@}

/** @name ADS1115 ADC Configuration */
///@{
#define ADS1115_ADDR (0x48 << 1)    //!< I2C address of the ADS1115 ADC.
#define ADS1115_REG_CONV   0x00     //!< ADS1115 Conversion Register.
#define ADS1115_REG_CONFIG 0x01     //!< ADS1115 Configuration Register.
#define ADS1115_PGA 0b001           //!< PGA setting for +/-4.096V range.
#define CONTINUOUS_MODE 1           //!< Set to 1 for continuous conversion, 0 for single-shot.
#define DIFFERENTIAL_MODE 1         //!< Set to 1 for differential mode (for current sensor).
///@}

/** @name System Limits */
///@{
#define MAX_TEMP_SENSORS 2     //!< Maximum number of temperature sensors per channel.
#define MAX_CHANNELS 5         //!< Maximum number of power channels.
#define MAX_FANS 1             //!< Maximum number of fan controllers.
#define MAX_EXTERNAL_BUTTONS 1 //!< Maximum number of external(not linked to specific channel) buttons.
///@}

/** @name Sensor and Channel Configuration */
///@{
#define TEMPERATURE_ERROR -30           //!< Temperature value indicating a sensor error.
#define NAME_LENGTH 4                   //!< Maximum length for a channel name (excluding null terminator).
///@}

#endif /* INC_DEFINES_H_ */
