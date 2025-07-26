/*
 * controller.h
 *
 *  Created on: Jul 9, 2025
 *      Author: kiril
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "project_types.h"

extern I2C_HandleTypeDef hi2c1;

void init_controller(PowerChannel** ch, uint8_t ch_count, Button** buttons, uint8_t btn_count, FanController** fans);
void delay(uint32_t ms);

#endif /* INC_CONTROLLER_H_ */
