/**
 * @file controller.h
 * @brief Main application controller logic.
 * @author kiril
 * @date Jul 9, 2025
 *
 * @details This file defines the main initialization and delay functions
 *          for the power supply's central controller.
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "project_types.h"
#include <stdio.h>


/**
 * @brief Initializes the main controller, hardware peripherals, and internal data structures.
 *
 * This function sets up all required components for the power supply system, including:
 * - power channels (PWM, ADC, etc.)
 * - buttons
 * - fan controllers
 * - LCD display
 * - external and internal ADCs
 *
 * It must be called once during startup after HAL initialization.
 *
 * @param[in] ch          Array of pointers to power channel structures.
 * @param[in] ch_count    Number of power channels.
 * @param[in] buttons     Array of pointers to button structures.
 * @param[in] btn_count   Number of buttons.
 * @param[in] fans        Array of pointers to fan controller structures.
 * @param[in] hi2c        Pointer to the I2C handle used for external devices (LCD, ADS1115).
 * @param[in] hadc        Pointer to the internal ADC handle.
 */
void init_controller(PowerChannel** ch, uint8_t ch_count, Button** buttons, uint8_t btn_count, FanController** fans, uint8_t fans_count, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc);

/**
 * @brief A custom delay function that continues to process background tasks.
 * @details This delay calls the main `routine()` function in a loop, ensuring
 *          that sensor readings and button updates continue during the delay.
 * @param ms The delay duration in milliseconds.
 */
void delay(uint32_t ms, bool skip_refresh);

/**
 * @brief The main application loop.
 * @details This function should be called repeatedly in the `while(1)` loop of `main.c`.
 *          It handles sensor updates, button actions, and screen updates.
 */
void main_loop(void);

#endif /* INC_CONTROLLER_H_ */
