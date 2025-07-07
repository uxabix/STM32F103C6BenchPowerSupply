/*
 * defines.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

#define LCD_ADDR (0x27 << 1)
#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#define LCD_DELAY_MS 5

#define ADS1115_ADDR (0x48 << 1)
#define ADS1115_REG_CONV   0x00
#define ADS1115_REG_CONFIG 0x01
#define ADS1115_PGA 0b001
#define CONTINUOUS_MODE 1  // Установи в 0 для single-shot
#define NON_DIFFERENTIAL_MODE 0

#define MAX_TEMP_SENSORS 2  // можно менять
#define MAX_CHANNELS 5

// Коэффициент пересчета АЦП ADS1115 -> ток (в А)
// Для 0.05 Ом и усиления по умолчанию: 1 bit = ~0.000125V / 0.05Ω = 2.5 mA
#define CURRENT_CONVERSION_FACTOR 0.000125f / 0.05f  // = 0.0025 (примерно)
#define TEMPERATURE_ERROR -30

#endif /* INC_DEFINES_H_ */
