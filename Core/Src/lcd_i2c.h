/*
 * lcd_i2c.h
 *
 *  Created on: Jun 29, 2025
 *      Author: kiril
 */

#ifndef SRC_LCD_I2C_H_
#define SRC_LCD_I2C_H_

#include "stm32f1xx_hal.h"

// Укажи свой адрес модуля (обычно 0x27 или 0x3F)
#define LCD_I2C_ADDRESS (0x27 << 1)

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_string(char *str);
void lcd_put_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);

#endif

