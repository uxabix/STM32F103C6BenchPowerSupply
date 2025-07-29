/*
 * buttons.h
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include "project_types.h"

/**
 * @brief Print the current button event for debugging.
 *
 * @param btn Pointer to the Button structure.
 */
void print_event(Button* btn);


/**
 * @brief Update the state of a single button based on current time and pin state.
 *
 * @param btn Pointer to the Button structure.
 * @param current_time Current system time in milliseconds (e.g., HAL_GetTick()).
 */
void button_update(Button* btn, uint32_t current_time);

/**
 * @brief Update all buttons in the array.
 *
 * @param buttons Array of pointers to Button structures.
 * @param count Number of buttons in the array.
 */
void update_buttons(Button** buttons, uint8_t count);

#endif /* INC_BUTTONS_H_ */
