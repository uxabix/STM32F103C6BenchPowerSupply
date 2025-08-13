/**
 * @file controller.c
 * @brief Main application controller logic for the power supply.
 * @author kiril
 * @date Jul 8, 2025
 *
 * @details This file contains the core logic for the power supply unit, including
 *          initialization, state management, sensor reading, button handling,
 *          and LCD screen updates.
 */

#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "controller_globals.h"
#include "controller.h"
#include "controller_screen.h"
#include "controller_buttons.h"
#include "controller_getset.h"
#include "defines.h"
#include "project_types.h"
#include "buttons.h"
#include "sensor_reader.h"


/**
 * @brief Initializes the main controller, peripherals, and data structures.
 */
void init_controller(PowerChannel** ch, uint8_t ch_count, Button** buttons, uint8_t btn_count, FanController** fans, uint8_t fans_count, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc){
	power_channels = ch;
	channels_count = ch_count;
	additional_buttons = buttons;
	additional_buttons_count = btn_count;
	fan_controllers = fans;
	fan_controllers_count = fans_count;

	// Populate helper arrays with pointers to channels that have specific features (temp, current, etc.) for efficient polling.
	set_controller_variables();

    // Wait for LCD to become ready
	while (HAL_I2C_IsDeviceReady(hi2c, LCD_ADDR, 3, HAL_MAX_DELAY) != HAL_OK) debug_printf("Waiting for lcd1602...\r\n");

	// Initialize LCD and custom characters
	controller_screen_init(hi2c);

    // Initialize ADC subsystem (internal and/or external)
	init_sensors(hadc, hi2c);
	debug_printf("Controller initialization finished\r\n");
}

/**
 * @brief Calculates a ratio from 0.0 to 1.0 based on where a value falls between a start and max threshold.
 * @param value The current input value.
 * @param start The value at which the ratio begins to be > 0.
 * @param max The value at which the ratio reaches 1.0.
 * @return A float from 0.0 to 1.0.
 */
static float interpolate_ratio(float value, float start, float max) {
	if (value <= start) return 0.0f;
	if (value >= max) return 1.0f;
	return (value - start) / (max - start);
}

/**
 * @brief Calculates the target fan speed ratio based on temperature readings.
 *
 * This function determines the highest normalized temperature across all channels,
 * where normalization is performed relative to each channel's shutdown threshold.
 * The resulting temperature "danger ratio" is then mapped to a fan speed ratio
 * in the range [0.0, 1.0] using the configured start and maximum temperature ratios.
 *
 * @param fan Pointer to the FanController instance.
 * @return Fan speed ratio in the range [0.0, 1.0].
 */
static float get_fan_ratio_temperature(FanController* fan)
{
    // Unused variable required by get_max_temp() and get_max_temp_by_channel() APIs
    int8_t dummy_result;

    // Determine the highest normalized temperature across all channels
    float max_ratio = get_max_temp_by_channel(get_max_temp(&dummy_result), &dummy_result);

    // Map the normalized temperature ratio to a fan speed ratio
    float fan_ratio = interpolate_ratio(max_ratio, fan->temp_start_ratio, fan->temp_max_ratio);

    return fan_ratio;
}

/**
 * @brief Calculates the target fan speed ratio based on current measurements.
 *
 * This function determines the highest normalized current across all channels,
 * where normalization is performed relative to the channel's shutdown threshold.
 * The resulting "danger ratio" is then mapped to a fan speed ratio in the range [0.0, 1.0]
 * using the configured start and maximum current ratios.
 *
 * @param fan Pointer to the FanController instance.
 * @return Fan speed ratio in the range [0.0, 1.0].
 */
static float get_fan_ratio_current(FanController* fan)
{
    // Find the channel with the highest measured current
    float max_current;
    PowerChannel* channel = get_max_current(&max_current);

    // Normalize the current value relative to the shutdown threshold of this channel
    float ratio = max_current / channel->current_sensor->shutdown_threshold;

    // Map the normalized current ratio to a fan speed ratio
    float fan_ratio = interpolate_ratio(ratio, fan->current_start_ratio, fan->current_max_ratio);

    return fan_ratio;
}

/**
 * @brief Updates the fan speed based on the most critical (highest) danger ratio.
 *
 * This function evaluates both temperature-based and current-based fan control modes
 * (if enabled), determines the maximum danger ratio, and updates the fan speed accordingly.
 * The computed speed ratio is converted into a PWM duty cycle and applied to the configured
 * PWM output. If the speed ratio is below the mechanical start threshold of the fan,
 * the output is clamped to ensure reliable startup.
 *
 * @param fan Pointer to the FanController to update.
 */
static void update_fan_speed(FanController* fan)
{
    if (!fan) return;

    float temp_ratio = 0.0f;
    if (fan->temp_activation) {
        temp_ratio = get_fan_ratio_temperature(fan);
    }

    float current_ratio = 0.0f;
    if (fan->current_activation) {
        current_ratio = get_fan_ratio_current(fan);
    }

    // Choose the higher of the two danger ratios
    float fan_ratio = (temp_ratio > current_ratio) ? temp_ratio : current_ratio;
    fan->current_speed = fan_ratio;

    // Apply PWM output if properly configured
    if (fan->pwm.type == OUTPUT_PWM && fan->pwm.pwm_timer != NULL) {
        uint32_t pwm_max = fan->pwm.pwm_timer->Init.Period;
        uint32_t value = (uint32_t)(fan_ratio * pwm_max);

        // Enforce minimum startup duty if ratio is above noise threshold but below start threshold
        if (fan_ratio > 0.01f && fan_ratio < (float)fan->min_start_speed) {
            value = (uint32_t)(fan->min_start_speed * pwm_max);
        }

        // Invert PWM signal if required
        if (fan->pwm.pwm_inversed) {
            value = pwm_max - value;
        }

        // Apply the computed PWM value
        __HAL_TIM_SET_COMPARE(fan->pwm.pwm_timer, fan->pwm.pwm_channel, value);
        fan->pwm.pwm_last_value = value;
    }
}


/**
 * @brief Updates the speed for all configured fan controllers.
 */
static void update_all_fans_speed() {
	for (uint8_t i = 0; i < fan_controllers_count; i++) {
		update_fan_speed(fan_controllers[i]);
	}
}

/**
 * @brief Performs all periodic background tasks.
 * @details This function is the heart of the non-blocking operation. It updates all
 *          sensors and buttons.
 */
static void routine(bool skip_refresh){
	// Reset warning and shutdown flags. They will be re-evaluated by the update functions.
	for (uint8_t i = 0; i < channels_count; i++){
		power_channels[i]->in_warning_state = false;
		power_channels[i]->in_shutdown_state = false;
	}
	update_temperatures(temp_channels, temp_channels_count);
	update_all_fans_speed();
	update_currents(current_channels, current_channels_count);
	update_voltages(voltage_channels, voltage_channels_count);
	update_buttons(buttons, buttons_count);
	buttons_action();
	// The refresh_screen flag can be set by other modules (e.g., button handlers) to request an immediate screen update.
	if (refresh_screen && !skip_refresh){
		refresh_screen = false;
		update_screen();
	}
}

/**
 * @brief Delay with non-blocking background processing.
 * @param ms Milliseconds to wait.
 * @param skip_refresh If true, suppress LCD refresh during delay.
 */
void delay(uint32_t ms, bool skip_refresh){
	uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < ms){
		routine(skip_refresh);
	}
}

void main_loop(){
	// The main loop consists of a delay during which background tasks are processed.
	delay(SCREEN_UPDATE_DELAY, false);
	// After the delay, explicitly update the screen with the latest data.
	update_screen();
}



