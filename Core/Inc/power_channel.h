/*
 * power_channel.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_POWER_CHANNEL_H_
#define INC_POWER_CHANNEL_H_

#include "project_types.h"

void init_pwm_channel(OutputControl* pwm_channel);
bool disactivate_channel(PowerChannel* channel);
bool activate_channel(PowerChannel* channel);
bool toggle_channel(PowerChannel* channel);

#endif /* INC_POWER_CHANNEL_H_ */
