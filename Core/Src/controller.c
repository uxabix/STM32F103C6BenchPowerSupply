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
#include "defines.h"
#include "project_types.h"
#include "buttons.h"
#include "sensor_reader.h"


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
static void init_pwm_channel(uint8_t i, OutputControl* pwm_channel){
	if (power_channels[i]->output.type == OUTPUT_PWM){
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

/**
 * @brief Loads the custom character bitmaps into the LCD's CGRAM.
 */

/**
 * @brief Initializes the main controller, peripherals, and data structures.
 */
void init_controller(PowerChannel** ch, uint8_t ch_count, Button** buttons, uint8_t btn_count, FanController** fans, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc){
	power_channels = ch;
	channels_count = ch_count;
	additional_buttons = buttons;
	additional_buttons_count = btn_count;
	fan_controllers = fans;

	// Populate sensor-specific and button-specific channel lists for polling
	for (uint8_t i = 0; i < channels_count; i++){
		add_temp_channels(i);
		add_current_channels(i);
		add_voltage_channels(i);
		add_pwm_channels(i);
		add_buttons_channels(i);
		init_pwm_channel(i, &power_channels[i]->output);
	}
	set_buttons();

    // Wait for LCD to become ready
	while (HAL_I2C_IsDeviceReady(hi2c, LCD_ADDR, 3, HAL_MAX_DELAY) != HAL_OK) debug_printf("Waiting for lcd1602...\r\n");

	// Initialize LCD and custom characters
	controller_screen_init(hi2c);

    // Initialize ADC subsystem (internal and/or external)
	init_sensors(hadc, hi2c);
	debug_printf("Controller initialization finished\r\n");
}

/**
 * @brief Performs all periodic background tasks.
 * @details This function is the heart of the non-blocking operation. It updates all
 *          sensors and buttons.
 */
static void routine(){
	// Reset warning and shutdown state for each channel before updating sensor values
	for (uint8_t i = 0; i < channels_count; i++){
		power_channels[i]->in_warning_state = false;
		power_channels[i]->in_shutdown_state = false;
	}
	update_temperatures(temp_channels, temp_channels_count);
	update_currents(current_channels, current_channels_count);
	update_voltages(voltage_channels, voltage_channels_count);
	update_buttons(buttons, buttons_count);
	buttons_action();
}

void delay(uint32_t ms){
	uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < ms){
		routine();
	}
}

void main_loop(){
	routine();
	delay(500);
	update_screen();
}



