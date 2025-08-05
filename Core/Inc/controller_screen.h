/**
 * @file controller_screen.h
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
 *
 * This function sets up the LCD display connected via I2C,
 * initializes the hardware interface, and loads any custom
 * characters or symbols needed for the UI.
 *
 * @param hi2c Pointer to the I2C handle used for LCD communication.
 */
void controller_screen_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Updates the currently displayed screen based on system state.
 *
 * This function should be called periodically or when a screen refresh
 * is required. It will handle updating all displayed information
 * including sensor readings, warnings, and menu navigation.
 */
void update_screen();

#endif /* INC_CONTROLLER_SCREEN_H_ */
