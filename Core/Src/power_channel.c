/*
 * power_channel.c
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#include <stdbool.h>
#include "power_channel.h"

void init_pwm_channel(OutputControl* pwm_channel){
	  HAL_TIM_PWM_Start(pwm_channel->pwm_timer, pwm_channel->pwm_channel);
}

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

PowerChannel* get_channel_with_max_current() {
    PowerChannel* max_channel = NULL;
    float max_current = -1.0f;

    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        PowerChannel* ch = &channels[i];
        if (ch->current_sensor == NULL) continue;
        float current = ch->current_sensor->last_value;
        if (ch->current_sensor != NULL && current > max_current) {
        	max_current = current;
			max_channel = ch;
        }
    }

    printf("Max current x100: %d\r\n", (int)(max_current * 100));
    return max_channel;  // NULL, если ни у одного канала нет датчика тока
}

PowerChannel* get_channel_with_max_voltage() {
    PowerChannel* max_channel = NULL;
    float max_voltage = -1.0f;

    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        PowerChannel* ch = &channels[i];
        if (ch->voltage_sensor == NULL) continue;
        float voltage = ch->voltage_sensor->last_value;
        if (voltage > max_voltage) {
        	max_voltage = voltage;
			max_channel = ch;
        }
    }
    printf("Max voltage x100: %d\r\n", (int)(max_voltage * 100));
    return max_channel;  // NULL, если ни один канал не измеряет напряжение
}
