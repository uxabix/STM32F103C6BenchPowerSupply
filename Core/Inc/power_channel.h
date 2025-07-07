/*
 * power_channel.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_POWER_CHANNEL_H_
#define INC_POWER_CHANNEL_H_

#include "project_types.h"

extern PowerChannel channels[MAX_CHANNELS];

PowerChannel* get_channel_with_max_current();
PowerChannel* get_channel_with_max_voltage();

#endif /* INC_POWER_CHANNEL_H_ */
