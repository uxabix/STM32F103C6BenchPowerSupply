/**
 * @file power_channel.h
 * @brief Functions for controlling the state of a power channel.
 * @author kiril
 * @date Jul 7, 2025
 */

#ifndef INC_POWER_CHANNEL_H_
#define INC_POWER_CHANNEL_H_

#include "project_types.h"

/**
 * @brief Deactivates a power channel's output.
 * @param channel Pointer to the PowerChannel to deactivate.
 * @return The new state of the channel (always false).
 */
bool disactivate_channel(PowerChannel* channel);

/**
 * @brief Activates a power channel's output if it is not in a shutdown state.
 * @param channel Pointer to the PowerChannel to activate.
 * @return true if the channel was successfully enabled, false otherwise.
 */
bool activate_channel(PowerChannel* channel);

/**
 * @brief Toggles the state of a power channel if it is not in a shutdown state.
 * @param channel Pointer to the PowerChannel to toggle.
 * @return The new state of the channel (true if enabled, false if disabled).
 */
bool toggle_channel(PowerChannel* channel);

#endif /* INC_POWER_CHANNEL_H_ */
