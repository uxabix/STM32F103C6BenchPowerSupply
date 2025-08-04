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

	// Populate sensor-specific and button-specific channel lists for polling
	set_controller_variables();

    // Wait for LCD to become ready
	while (HAL_I2C_IsDeviceReady(hi2c, LCD_ADDR, 3, HAL_MAX_DELAY) != HAL_OK) debug_printf("Waiting for lcd1602...\r\n");

	// Initialize LCD and custom characters
	controller_screen_init(hi2c);

    // Initialize ADC subsystem (internal and/or external)
	init_sensors(hadc, hi2c);
	debug_printf("Controller initialization finished\r\n");
}

static float interpolate_ratio(float value, float start, float max) {
	if (value <= start) return 0.0f;
	if (value >= max) return 1.0f;
	return (value - start) / (max - start);
}


/**
 * @brief Updates the fan speed based on the highest temperature-to-shutdown-threshold ratio.
 * @param fan Pointer to the FanController to update.
 * @param channels Array of all power channels.
 * @param channel_count Number of channels in the array.
 */
static void update_fan_speed(FanController* fan)
{
    if (!fan)  return;

    int8_t dummy_result; // Value unused
    float max_ratio = get_max_temp_by_channel(get_max_temp(&dummy_result), &dummy_result);

    float fan_ratio = interpolate_ratio(max_ratio, fan->start_ratio, fan->max_ratio);
    fan->current_speed = fan_ratio;

    // Apply PWM output if configured
    if (fan->pwm.type == OUTPUT_PWM && fan->pwm.pwm_timer != NULL) {
        uint32_t pwm_max = fan->pwm.pwm_timer->Init.Period;
        uint32_t value = fan_ratio * pwm_max;

        if (fan->pwm.pwm_inversed) {
            value = pwm_max - value;
        }

        __HAL_TIM_SET_COMPARE(fan->pwm.pwm_timer, fan->pwm.pwm_channel, value);
        fan->pwm.pwm_last_value = value;
    }
}

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
	// Reset warning and shutdown state for each channel before updating sensor values
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
	delay(SCREEN_UPDATE_DELAY, false);
	update_screen();
}



