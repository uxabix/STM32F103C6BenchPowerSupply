/*
 * power_channel.c
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#include <stdbool.h>
#include "power_channel.h"

/*
 * Disactivates channel output
 */
bool disactivate_channel(PowerChannel* channel){
	if (channel->output.type == OUTPUT_GPIO){
		HAL_GPIO_WritePin(channel->output.pin.port, channel->output.pin.pin, channel->output.active_high == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else if (channel->output.type == OUTPUT_PWM){
		__HAL_TIM_SET_COMPARE(channel->output.pwm_timer, channel->output.pwm_channel, channel->output.pwm_inversed ? 0 : channel->output.pwm_timer->Init.Period);
	}
	channel->enabled = false;

	return false;
}

/*
 * Activates channel output if it isn't in shutdown state
 * Returns true if channel was enabled and false when not
 */
bool activate_channel(PowerChannel* channel){
	if (channel->in_shutdown_state) return false;
	if (channel->output.type == OUTPUT_GPIO){
		HAL_GPIO_WritePin(channel->output.pin.port, channel->output.pin.pin, channel->output.active_high == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else if (channel->output.type == OUTPUT_PWM){
		__HAL_TIM_SET_COMPARE(channel->output.pwm_timer, channel->output.pwm_channel, channel->output.pwm_last_value);
	}
	channel->enabled = true;

	return true;
}

/*
 * If channel not in shutdown state changes the state of channel to opposite
 * Returns true if channel was enabled and false when not
 */
bool toggle_channel(PowerChannel* channel){
	if (channel->in_shutdown_state) return false;
	if (channel->enabled){
		disactivate_channel(channel);
	} else {
		activate_channel(channel);
	}

	return channel->enabled;
}
