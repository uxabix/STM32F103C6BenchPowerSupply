/**
 * @file controller_getset.h
 * @brief Data initialization and status retrieval interface for power channels.
 *
 * Provides functions to initialize controller-related structures and retrieve
 * temperature, current, and voltage status from all configured power channels.
 * Useful for diagnostics, alert generation, and dynamic display updates.
 *
 * @author kiril
 * @date August 4, 2025
 */

#ifndef INC_CONTROLLER_GETSET_H_
#define INC_CONTROLLER_GETSET_H_

#include <inttypes.h>

#include "project_types.h"


/**
 * @brief Initializes and populates controller-wide data structures.
 * @details This function scans the main `power_channels` array and populates specialized
 *          lists (e.g., `temp_channels`, `pwm_channels`) for more efficient processing.
 *          It also initializes PWM outputs and aggregates all buttons into a master list.
 */
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
 * @brief Prepares the list of alert-triggering channels (warning/shutdown) for display.
 *
 * Should be called during UI refresh or status evaluation to ensure up-to-date visuals.
 */
void set_alert_channels();

#endif /* INC_CONTROLLER_GETSET_H_ */
