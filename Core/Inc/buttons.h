/**
 * @file buttons.h
 * @brief Handles button state detection including debouncing, short and long presses.
 * @author kiril
 * @date Jul 8, 2025
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include "project_types.h"
#include <stdio.h>

/**
 * @brief Prints the current button event for debugging purposes.
 * @param btn Pointer to the Button structure to inspect.
 */
void print_event(Button* btn);


/**
 * @brief Updates the state machine for a single button.
 * @details This function implements debouncing and detects short and long press events.
 *          It should be called periodically for each button.
 * @param btn Pointer to the Button structure to update.
 * @param current_time The current system time in milliseconds (e.g., from HAL_GetTick()).
 */
void button_update(Button* btn, uint32_t current_time);

/**
 * @brief Updates all buttons in a given array.
 * @details This is a convenience function that iterates through an array of buttons
 *          and calls button_update() for each one.
 * @param buttons An array of pointers to Button structures.
 * @param count The number of buttons in the array.
 */
void update_buttons(Button** buttons, uint8_t count);

#endif /* INC_BUTTONS_H_ */
