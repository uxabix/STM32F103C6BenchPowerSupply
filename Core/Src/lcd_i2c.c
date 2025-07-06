#include "lcd_i2c.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"

extern I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef *_lcd_i2c;

#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04
#define READ_WRITE 0x02
#define REGISTER_SELECT 0x01

static void lcd_send_cmd(uint8_t cmd);
static void lcd_send_data(uint8_t data);
static void lcd_send(uint8_t data, uint8_t mode);
static void lcd_send_4bits(uint8_t data);

void lcd_init(I2C_HandleTypeDef *hi2c) {
    _lcd_i2c = hi2c;
    HAL_Delay(50);

    lcd_send_cmd(0x33);
    lcd_send_cmd(0x32);
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x0C);
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x01);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_put_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(addr);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str++));
    }
}

static void lcd_send_cmd(uint8_t cmd) {
    lcd_send(cmd, 0);
}

static void lcd_send_data(uint8_t data) {
    lcd_send(data, REGISTER_SELECT);
}

static void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t high = (data & 0xF0) | LCD_BACKLIGHT | mode;
    uint8_t low  = ((data << 4) & 0xF0) | LCD_BACKLIGHT | mode;

    lcd_send_4bits(high);
    lcd_send_4bits(low);
}

static void lcd_send_4bits(uint8_t data) {
    HAL_I2C_Master_Transmit(_lcd_i2c, LCD_I2C_ADDRESS, &data, 1, 10);
    HAL_Delay(1);
    uint8_t en = data | ENABLE;
    HAL_I2C_Master_Transmit(_lcd_i2c, LCD_I2C_ADDRESS, &en, 1, 10);
    HAL_Delay(1);
    en = data & ~ENABLE;
    HAL_I2C_Master_Transmit(_lcd_i2c, LCD_I2C_ADDRESS, &en, 1, 10);
    HAL_Delay(1);
}
