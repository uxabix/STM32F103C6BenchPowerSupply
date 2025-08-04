/*
 * controller_getset.c
 *
 *  Created on: Aug 4, 2025
 *      Author: kiril
 */
#include <inttypes.h>
#include <stdlib.h>

#include "controller_getset.h"
#include "controller_globals.h"
#include "project_types.h"

/** @brief Maps custom character symbols to their CGRAM locations. */

/**
 * @brief Adds a channel to the list of channels with temperature sensors.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void add_temp_channels(uint8_t i){
	if (power_channels[i]->temp_sensor_count > 0){
		temp_channels_count += 1;
		temp_channels = realloc(temp_channels, temp_channels_count * sizeof(PowerChannel*));
		temp_channels[temp_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Adds a channel to the list of channels with current sensors.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void add_current_channels(uint8_t i){
	if (power_channels[i]->current_sensor != NULL){
		current_channels_count += 1;
		current_channels = realloc(current_channels, current_channels_count * sizeof(PowerChannel*));
		current_channels[current_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Adds a channel to the list of channels with voltage sensors.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void add_voltage_channels(uint8_t i){
	if (power_channels[i]->voltage_sensor != NULL){
		voltage_channels_count += 1;
		voltage_channels = realloc(voltage_channels, voltage_channels_count * sizeof(PowerChannel*));
		voltage_channels[voltage_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Adds a channel to the list of channels with pwm type output.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void add_pwm_channels(uint8_t i){
	if (power_channels[i]->output.type == OUTPUT_PWM){
		pwm_channels_count += 1;
		pwm_channels = realloc(pwm_channels, pwm_channels_count * sizeof(PowerChannel*));
		pwm_channels[pwm_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Adds a channel to the list of channels with buttons.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void add_buttons_channels(uint8_t i){
	if (power_channels[i]->button != NULL){
		button_channels_count += 1;
		button_channels = realloc(button_channels, button_channels_count * sizeof(PowerChannel*));
		button_channels[button_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Starts the PWM timer for a channel if its output is PWM.
 * @param i Index of the channel in the main `power_channels` array.
 * @param pwm_channel Pointer to the channel's OutputControl struct.
 */
static void init_pwm_channel(OutputControl* pwm_channel){
	if (pwm_channel->type == OUTPUT_PWM){
		HAL_TIM_PWM_Start(pwm_channel->pwm_timer, pwm_channel->pwm_channel);
	}
}

/**
 * @brief Populates the master `buttons` array from channel buttons and additional buttons.
 */
static void set_buttons(){
	for (uint8_t i = 0; i < button_channels_count; i++){
		buttons_count += 1;
		buttons = realloc(buttons, buttons_count * sizeof(Button*));
		buttons[buttons_count - 1] = power_channels[i]->button;
	}
	for (uint8_t i = 0; i < additional_buttons_count; i++){
		buttons_count += 1;
		buttons = realloc(buttons, buttons_count * sizeof(Button));
		buttons[buttons_count - 1] = additional_buttons[i];
	}
}

void set_controller_variables(){
	// Populate sensor-specific and button-specific channel lists for polling
	for (uint8_t i = 0; i < channels_count; i++){
		add_temp_channels(i);
		add_current_channels(i);
		add_voltage_channels(i);
		add_pwm_channels(i);
		add_buttons_channels(i);
		init_pwm_channel(&power_channels[i]->output);
	}
	for (uint8_t i = 0; i < fan_controllers_count; i++){
		init_pwm_channel(&fan_controllers[i]->pwm);
	}
	set_buttons();
}

/**
 * @brief Gets the maximum temperature and its "danger ratio" for a given channel.
 * @param ch The channel to check.
 * @param[out] value Pointer to store the highest temperature value found.
 * @return The ratio of the highest temperature to its shutdown threshold.
 */
float get_max_temp_by_channel(const PowerChannel* ch, int8_t *value){
	float max_ratio = -99.0f;
	int8_t temp = -99.0f;

	for (uint8_t i = 0; i < ch->temp_sensor_count; i++){
		float ratio = 1.0f * ch->temp_sensors[i].last_value / ch->temp_sensors[i].shutdown_threshold;
		if (ratio > max_ratio){
			max_ratio = ratio;
			temp = ch->temp_sensors[i].last_value;
		}
	}

	(*value) = temp;
	return max_ratio;
}

/**
 * @brief Finds the channel with the highest temperature relative to its limit.
 * @param[out] result Pointer to store the temperature value of the hottest channel.
 * @return A pointer to the hottest power channel.
 */
PowerChannel* get_max_temp(int8_t* result){
	PowerChannel* ch = NULL;
	float max_ratio = -99.0f;
	float temp_ratio = -99.0f;
	int8_t temp = -99.0f;
	int8_t max_temp = -99.0f;
	for (uint8_t i = 0; i < temp_channels_count; i++){
		temp_ratio = get_max_temp_by_channel(temp_channels[i], &temp);
		if (temp_ratio > max_ratio){
			max_ratio = temp_ratio;
			max_temp = temp;
			ch = temp_channels[i];
		}
	}

	*result = max_temp;
	return ch;
}

/**
 * @brief Finds the channel with the highest current relative to its limit.
 * @param[out] result Pointer to store the current value of the highest-current channel.
 * @return A pointer to the power channel with the highest current.
 */
PowerChannel* get_max_current(float* result){
	PowerChannel* ch = NULL;
	float max_current = -99.0f;
	float max_ratio = -99.0f;
	float temp_ratio;
	for (uint8_t i = 0; i < current_channels_count; i++){
		temp_ratio = current_channels[i]->current_sensor->last_value / current_channels[i]->current_sensor->shutdown_threshold;
		if (temp_ratio > max_ratio){
			ch = current_channels[i];
			max_ratio = temp_ratio;
			max_current = current_channels[i]->current_sensor->last_value;
		}
	}

	*result = max_current;
	return ch;
}

/**
 * @brief Finds the channel with the voltage furthest from its nominal range.
 * @param[out] result Pointer to store the voltage value of the channel.
 * @return A pointer to the power channel with the most critical voltage.
 */
PowerChannel* get_max_voltage(float* result){
	PowerChannel* ch = NULL;
	float voltage = -99.0f;
	float max_ratio = -99.0f;
	float temp_ratio_overvoltage;
	float temp_ratio_undervoltage;
	float temp_ratio;
	for (uint8_t i = 0; i < voltage_channels_count; i++){
		temp_ratio_overvoltage = voltage_channels[i]->voltage_sensor->last_value / voltage_channels[i]->voltage_sensor->overvoltage_threshold;
		temp_ratio_undervoltage = voltage_channels[i]->voltage_sensor->undervoltage_threshold / voltage_channels[i]->voltage_sensor->last_value;
		temp_ratio = temp_ratio_overvoltage > temp_ratio_undervoltage ? temp_ratio_overvoltage : temp_ratio_undervoltage;
		if (temp_ratio > max_ratio){
			ch = voltage_channels[i];
			max_ratio = temp_ratio;
			voltage = voltage_channels[i]->voltage_sensor->last_value;
		}
	}

	*result = voltage;
	return ch;
}

/**
 * @brief Adds a channel to the list of channels in a warning state.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void set_warning_channel(uint8_t i){
	if (power_channels[i]->in_warning_state){
		warning_channels_count += 1;
		warning_channels = realloc(warning_channels, warning_channels_count * sizeof(PowerChannel));
		warning_channels[warning_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Adds a channel to the list of channels in a shutdown state.
 * @param i Index of the channel in the main `power_channels` array.
 */
static void set_shutdown_channel(uint8_t i){
	if (power_channels[i]->in_shutdown_state){
		shutdown_channels_count += 1;
		shutdown_channels = realloc(shutdown_channels, shutdown_channels_count * sizeof(PowerChannel));
		shutdown_channels[shutdown_channels_count - 1] = power_channels[i];
	}
}

/**
 * @brief Populates the warning and shutdown channel lists for display purposes.
 */
void set_alert_channels(){
	warning_channels_count = 0;
	shutdown_channels_count = 0;
	for (uint8_t i = 0; i < channels_count; i++){
		set_warning_channel(i);
		set_shutdown_channel(i);
	}
}
