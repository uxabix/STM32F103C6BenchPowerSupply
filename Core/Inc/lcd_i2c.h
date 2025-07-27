/*
 * lcd_i2c.h
 *
 *  Created on: Jun 29, 2025
 *      Author: kiril
 */

#ifndef SRC_LCD_I2C_H_
#define SRC_LCD_I2C_H_

#include "stm32f1xx_hal.h"

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint8_t lcd_addr, uint8_t data);
void LCD_CreateChar(uint8_t lcd_addr, uint8_t location, const uint8_t charmap[]);
void LCD_Init(I2C_HandleTypeDef* i2c, uint8_t lcd_addr);
void LCD_SendString(uint8_t lcd_addr, char *str);
void LCD_Clear(uint8_t lcd_addr);
void LCD_SetFirstLine(uint8_t lcd_addr);
void LCD_SetSecondLine(uint8_t lcd_addr);

#endif

