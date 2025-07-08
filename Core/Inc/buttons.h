/*
 * buttons.h
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include "project_types.h"

void button_update(Button* btn, uint32_t current_time);
void update_buttons(Button* buttons, uint8_t count);

#endif /* INC_BUTTONS_H_ */
