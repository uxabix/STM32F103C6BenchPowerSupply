/**
 * @file lcd_i2c.c
 * @brief Driver implementation for HD44780-compatible LCD with I2C backpack.
 * @author kiril
 * @date Jun 29, 2025
 */

#include "lcd_i2c.h"
#include "defines.h"
#include "controller.h" // For the custom delay() function

/** @brief Global pointer to the I2C handle used for communication. */
static I2C_HandleTypeDef* m_hi2c;

/**
 * @brief Sends a byte to the LCD via the I2C backpack.
 * @details This is the core communication function. It splits the byte into two
 *          4-bit nibbles and sends them sequentially, toggling the EN pin.
 * @param lcd_addr The I2C address of the LCD backpack.
 * @param data The byte to send (either a command or character data).
 * @param flags Control flags (PIN_RS for data, 0 for command).
 * @return HAL status of the I2C transmission.
 */
static HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags) {
	if (m_hi2c == NULL) return 0; // Safety check: return 0 if m_hi2c has not been initialized
    HAL_StatusTypeDef res;
    // Wait until the I2C device is ready
    while (HAL_I2C_IsDeviceReady(m_hi2c, lcd_addr, 1, HAL_MAX_DELAY) != HAL_OK);

    // Split data into high and low nibbles
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;

    // Data is sent in a 4-byte sequence to toggle the EN pin for each nibble
    uint8_t data_arr[4];
    data_arr[0] = high_nibble | flags | BACKLIGHT | PIN_EN; // Step 1: Send high nibble with EN high
    data_arr[1] = high_nibble | flags | BACKLIGHT;          // Step 2: Send same data with EN low to latch
    data_arr[2] = low_nibble  | flags | BACKLIGHT | PIN_EN; // Step 3: Send low nibble with EN high
    data_arr[3] = low_nibble  | flags | BACKLIGHT;          // Step 4: Send same data with EN low to latch

    res = HAL_I2C_Master_Transmit(m_hi2c, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
    delay(LCD_DELAY_MS, 1); // Use custom delay to keep tasks running
    return res;
}

/**
 * @brief Sends a command byte to the LCD.
 */
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
    LCD_SendInternal(lcd_addr, cmd, 0);
}

/**
 * @brief Sends a data byte (a character) to the LCD.
 */
void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

/**
 * @brief Creates a custom character and stores it in the LCD's CGRAM.
 */
void LCD_CreateChar(uint8_t lcd_addr, uint8_t location, const uint8_t charmap[]) {
    location &= 0x07; // Ensure location is within 0-7
    LCD_SendCommand(lcd_addr, 0x40 | (location << 3)); // Set CGRAM address
    for (int i = 0; i < 8; i++) {
        LCD_SendData(lcd_addr, charmap[i]);
    }
    LCD_SendCommand(lcd_addr, 0x80); // Return to DDRAM address
}

/**
 * @brief Initializes the LCD in 4-bit mode.
 */
void LCD_Init(I2C_HandleTypeDef* hi2c, uint8_t lcd_addr) {
	if (hi2c == NULL) {
	    printf("ERROR: hi2c is NULL in LCD_Init!\r\n");
	    return;
	}
	m_hi2c = hi2c;
    delay(50, 1); // Wait for LCD to power up properly (>40ms)

    // Initialization sequence for 4-bit mode
    LCD_SendCommand(lcd_addr, 0x30);
    delay(5, 1);
    LCD_SendCommand(lcd_addr, 0x30);
    delay(1, 1);
    LCD_SendCommand(lcd_addr, 0x30);
    delay(1, 1);
    LCD_SendCommand(lcd_addr, 0x20); // Set 4-bit mode
    delay(1, 1);

    // Configure display settings
    LCD_SendCommand(lcd_addr, 0x28); // Function Set: 4-bit, 2-line, 5x8 dots
    delay(1, 1);
    LCD_SendCommand(lcd_addr, 0x08); // Display OFF
    delay(1, 1);
    LCD_SendCommand(lcd_addr, 0x01); // Clear Display
    delay(2, 1);
    LCD_SendCommand(lcd_addr, 0x06); // Entry Mode Set: Increment cursor, no shift
    delay(1, 1);
    LCD_SendCommand(lcd_addr, 0x0C); // Display ON, Cursor OFF, Blink OFF
}

/**
 * @brief Sends a null-terminated string to be displayed on the LCD.
 */
void LCD_SendString(uint8_t lcd_addr, char *str) {
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

/**
 * @brief Clears the entire LCD display and returns the cursor to home (0,0).
 */
void LCD_Clear(uint8_t lcd_addr){
	LCD_SendCommand(lcd_addr, 0x01); // Clear display command
    delay(2, 1); // This command takes longer
}

/**
 * @brief Moves the cursor to the beginning of the first line.
 */
void LCD_SetFirstLine(uint8_t lcd_addr){
	LCD_SendCommand(lcd_addr, 0x80); // Set DDRAM address to 0x00
}

/**
 * @brief Moves the cursor to the beginning of the second line.
 */
void LCD_SetSecondLine(uint8_t lcd_addr){
	LCD_SendCommand(lcd_addr, 0xC0); // Set DDRAM address to 0x40
}
