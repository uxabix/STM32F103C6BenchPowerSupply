/**
 * @file lcd_i2c.h
 * @brief Driver for a standard HD44780-compatible LCD with an I2C backpack (PCF8574).
 * @author kiril
 * @date Jun 29, 2025
 */

#ifndef SRC_LCD_I2C_H_
#define SRC_LCD_I2C_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

/**
 * @brief Initializes the LCD in 4-bit mode.
 * @param i2c Pointer to the initialized I2C handle.
 * @param lcd_addr The I2C address of the LCD backpack.
 */
void LCD_Init(I2C_HandleTypeDef* i2c, uint8_t lcd_addr);

/**
 * @brief Sends a command byte to the LCD.
 * @param lcd_addr The I2C address of the LCD backpack.
 * @param cmd The command byte to send.
 */
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);

/**
 * @brief Sends a data byte (a character) to the LCD.
 * @param lcd_addr The I2C address of the LCD backpack.
 * @param data The character to display.
 */
void LCD_SendData(uint8_t lcd_addr, uint8_t data);

/**
 * @brief Creates a custom character and stores it in the LCD's CGRAM.
 * @param lcd_addr The I2C address of the LCD backpack.
 * @param location The CGRAM location to store the character (0-7).
 * @param charmap A pointer to an 8-byte array defining the character's 5x8 pixel bitmap.
 */
void LCD_CreateChar(uint8_t lcd_addr, uint8_t location, const uint8_t charmap[]);

/**
 * @brief Sends a null-terminated string to be displayed on the LCD.
 * @param lcd_addr The I2C address of the LCD backpack.
 * @param str Pointer to the string to send.
 */
void LCD_SendString(uint8_t lcd_addr, char *str);

/**
 * @brief Clears the entire LCD display and returns the cursor to home (0,0).
 * @param lcd_addr The I2C address of the LCD backpack.
 */
void LCD_Clear(uint8_t lcd_addr);

/**
 * @brief Moves the cursor to the beginning of the first line (address 0x80).
 * @param lcd_addr The I2C address of the LCD backpack.
 */
void LCD_SetFirstLine(uint8_t lcd_addr);

/*
 * @brief Set cursor to the beginning of the second line.
 */
void LCD_SetSecondLine(uint8_t lcd_addr);

#endif

