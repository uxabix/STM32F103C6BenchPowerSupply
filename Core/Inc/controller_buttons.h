/**
 * @file controller_buttons.h
 * @brief Interface for handling button events in the power supply controller firmware.
 *
 * Provides a function to process all registered buttons and dispatch their events
 * according to the current application state (main, channel, or settings view).
 *
 * @author kiril
 * @date August 3, 2025
 */

#ifndef INC_CONTROLLER_BUTTONS_H_
#define INC_CONTROLLER_BUTTONS_H_

/**
 * @brief Processes all button events and dispatches them based on application state.
 *
 * This function iterates over all configured buttons (both channel-related and additional),
 * checks their state for events (e.g., short press, long press), and applies the corresponding
 * action depending on the global `state` and `state_settings`.
 */
void buttons_action();

#endif /* INC_CONTROLLER_BUTTONS_H_ */
