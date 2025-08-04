/*
 * controller_getset.h
 *
 *  Created on: Aug 4, 2025
 *      Author: kiril
 */

#ifndef INC_CONTROLLER_GETSET_H_
#define INC_CONTROLLER_GETSET_H_

#include <inttypes.h>

#include "project_types.h"


void set_controller_variables();

/**
 * @brief Gets the maximum temperature and its "danger ratio" for a given channel.
 * @param ch The channel to check.
 * @param[out] value Pointer to store the highest temperature value found.
 * @return The ratio of the highest temperature to its shutdown threshold.
 */
float get_max_temp_by_channel(const PowerChannel* ch, int8_t *value);

/**
 * @brief Finds the channel with the highest temperature relative to its limit.
 * @param[out] result Pointer to store the temperature value of the hottest channel.
 * @return A pointer to the hottest power channel.
 */
PowerChannel* get_max_temp(int8_t* result);

/**
 * @brief Finds the channel with the highest current relative to its limit.
 * @param[out] result Pointer to store the current value of the highest-current channel.
 * @return A pointer to the power channel with the highest current.
 */
PowerChannel* get_max_current(float* result);

/**
 * @brief Finds the channel with the voltage furthest from its nominal range.
 * @param[out] result Pointer to store the voltage value of the channel.
 * @return A pointer to the power channel with the most critical voltage.
 */
PowerChannel* get_max_voltage(float* result);

/**
 * @brief Populates the warning and shutdown channel lists for display purposes.
 */
void set_alert_channels();

#endif /* INC_CONTROLLER_GETSET_H_ */
