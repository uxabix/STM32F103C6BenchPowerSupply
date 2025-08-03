/**
 * @file controller_screen.c
 * @brief LCD screen management for power supply controller.
 *
 * This module handles the logic for displaying data such as temperature,
 * current, voltage, warnings, and settings on the I2C LCD display.
 *
 * @author Kiril
 * @date August 3, 2025
 */

#ifndef INC_CONTROLLER_SCREEN_H_
#define INC_CONTROLLER_SCREEN_H_

/**
 * @brief Initializes the LCD screen and loads custom symbols.
 * @param hi2c Pointer to I2C handle used for LCD communication.
 * @param hadc Pointer to ADC handle (not used in this module, but passed for consistency).
 */
void controller_screen_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Updates the currently displayed screen based on system state.
 */
void update_screen();

#endif /* INC_CONTROLLER_SCREEN_H_ */
