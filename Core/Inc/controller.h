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

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Initializes the main controller.
 * @param ch Array of pointers to PowerChannel structures.
 * @param ch_count Number of power channels.
 * @param buttons Array of pointers to additional (non-channel) Button structures.
 * @param btn_count Number of additional buttons.
 * @param fans Array of pointers to FanController structures.
 */
void init_controller(PowerChannel** ch, uint8_t ch_count, Button** buttons, uint8_t btn_count, FanController** fans);

/**
 * @brief A custom delay function that continues to process background tasks.
 * @details This delay calls the main `routine()` function in a loop, ensuring
 *          that sensor readings and button updates continue during the delay.
 * @param ms The delay duration in milliseconds.
 */
void delay(uint32_t ms);

/**
 * @brief The main application loop.
 * @details This function should be called repeatedly in the `while(1)` loop of `main.c`.
 *          It handles sensor updates, button actions, and screen updates.
 */
void main_loop(void);

#endif /* INC_CONTROLLER_H_ */
