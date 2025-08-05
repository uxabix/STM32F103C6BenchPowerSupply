/**
 * @file power_channel.c
 * @brief Implements functions for controlling a power channel's output.
 * @author kiril
 * @date Jul 7, 2025
 */

#include <stdbool.h>
#include "power_channel.h"

/**
 * @brief Deactivates a power channel's output.
 * @details Sets the output GPIO to its inactive state or sets PWM duty cycle to 0.
 *          Updates the channel's `enabled` status to false.
 * @param channel Pointer to the PowerChannel to deactivate.
 * @return The new state of the channel (always false).
 */
bool disactivate_channel(PowerChannel* channel){
	if (channel->output.type == OUTPUT_GPIO){
        // Set pin to inactive state based on active_high logic
		HAL_GPIO_WritePin(channel->output.pin.port, channel->output.pin.pin,
                          channel->output.active_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else if (channel->output.type == OUTPUT_PWM){
        // Set PWM to off state (0% or 100% duty depending on inversion)
		uint32_t off_value = channel->output.pwm_inversed ? channel->output.pwm_timer->Init.Period : 0;
		__HAL_TIM_SET_COMPARE(channel->output.pwm_timer, channel->output.pwm_channel, off_value);
	}
	channel->enabled = false;

	return false;
}

/**
 * @brief Activates a power channel's output if it is not in a shutdown state.
 * @details Sets the output GPIO to its active state or restores the last PWM duty cycle.
 *          Updates the channel's `enabled` status to true.
 * @param channel Pointer to the PowerChannel to activate.
 * @return true if the channel was successfully enabled, false if it was in shutdown.
 */
bool activate_channel(PowerChannel* channel){
	if (channel->in_shutdown_state) {
        return false; // Cannot activate a channel that is in a shutdown state.
    }

	if (channel->output.type == OUTPUT_GPIO){
        // Set pin to active state based on active_high logic
		HAL_GPIO_WritePin(channel->output.pin.port, channel->output.pin.pin, //
                          channel->output.active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else if (channel->output.type == OUTPUT_PWM){
        // Restore the last active PWM value
		__HAL_TIM_SET_COMPARE(channel->output.pwm_timer, channel->output.pwm_channel, channel->output.pwm_last_value);
	}
	channel->enabled = true;

	return true;
}

/**
 * @brief Toggles the state of a power channel if it is not in a shutdown state.
 * @details If the channel is enabled, it will be disabled. If it is disabled, it
 *          will be enabled.
 * @param channel Pointer to the PowerChannel to toggle.
 * @return The new state of the channel (true if enabled, false if disabled or in shutdown).
 */
bool toggle_channel(PowerChannel* channel){
	if (channel->in_shutdown_state) {
        return false; // Cannot toggle a channel in shutdown state.
    }

	if (channel->enabled){
		return disactivate_channel(channel);
	}
	return activate_channel(channel);
}
