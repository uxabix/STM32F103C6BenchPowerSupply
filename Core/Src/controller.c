/*
 * controller.c
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include <string.h>
#include "controller.h"
#include "defines.h"
#include "sensor_reader.h"
#include "project_types.h"
#include "ftoa.h"
#include "lcd_i2c.h"
#include "buttons.h"

PowerChannel* power_channels = NULL;
uint8_t channels_count = 0;
Button* additional_buttons = NULL;
uint8_t additional_buttons_count = 0;
FanController* fan_controllers = NULL;
uint8_t fan_controllers_count = 0;
PowerChannel* temp_channels = NULL;
uint8_t temp_channels_count = 0;
PowerChannel* current_channels = NULL;
uint8_t current_channels_count = 0;
PowerChannel* voltage_channels = NULL;
uint8_t voltage_channels_count = 0;
PowerChannel* button_channels = NULL;
uint8_t button_channels_count = 0;
Button* buttons = NULL;
uint8_t buttons_count = 0;

PowerChannel* warning_channels = NULL;
uint8_t warning_channels_count = 0;
PowerChannel* shutdown_channels = NULL;
uint8_t shutdown_channels_count = 0;

typedef enum {
	State_Main,
	State_Channel,
	State_Settings
} ScreenState;
ScreenState state = State_Main;

void add_temp_channels(uint8_t i){
	if (power_channels[i].temp_sensor_count > 0){
		printf("Has temp!\r\n");
		temp_channels_count += 1;
		temp_channels = realloc(temp_channels, temp_channels_count * sizeof(PowerChannel));
		temp_channels[temp_channels_count - 1] = power_channels[i];
	}
}

void add_current_channels(uint8_t i){
	if (power_channels[i].current_sensor != NULL){
		printf("Has current!\r\n");
		current_channels_count += 1;
		current_channels = realloc(current_channels, current_channels_count * sizeof(PowerChannel));
		current_channels[current_channels_count - 1] = power_channels[i];
	}
}

void add_voltage_channels(uint8_t i){
	if (power_channels[i].voltage_sensor != NULL){
		printf("Has voltage!\r\n");
		voltage_channels_count += 1;
		voltage_channels = realloc(voltage_channels, voltage_channels_count * sizeof(PowerChannel));
		voltage_channels[voltage_channels_count - 1] = power_channels[i];
	}
}

void add_buttons_channels(uint8_t i){
	if (power_channels[i].button != NULL){
		printf("Has button!\r\n");
		button_channels_count += 1;
		button_channels = realloc(button_channels, button_channels_count * sizeof(PowerChannel));
		button_channels[button_channels_count - 1] = power_channels[i];
	}
}

void init_pwm_channel(uint8_t i, OutputControl* pwm_channel){
	if (power_channels[i].output.type == OUTPUT_PWM){
		HAL_TIM_PWM_Start(pwm_channel->pwm_timer, pwm_channel->pwm_channel);
	}
}

void set_buttons(){
	for (uint8_t i = 0; i < button_channels_count; i++){
		printf("New button!\r\n");
		buttons_count += 1;
		buttons = realloc(buttons, buttons_count * sizeof(PowerChannel));
		buttons[buttons_count - 1] = *power_channels[i].button;
	}
	for (uint8_t i = 0; i < additional_buttons_count; i++){
		printf("New button!\r\n");
		buttons_count += 1;
		buttons = realloc(buttons, buttons_count * sizeof(PowerChannel));
		buttons[buttons_count - 1] = additional_buttons[i];
	}
}

void init_controller(PowerChannel* ch, uint8_t ch_count, Button* buttons, uint8_t btn_count, FanController* fans){
	printf("Controller initialization started\r\n");
	power_channels = ch;
	channels_count = ch_count;
	additional_buttons = buttons;
	additional_buttons_count = btn_count;
	fan_controllers = fans;

	for (uint8_t i = 0; i < channels_count; i++){
		add_temp_channels(i);
		add_current_channels(i);
		add_voltage_channels(i);
		add_buttons_channels(i);
		init_pwm_channel(i, &power_channels[i].output);
	}
	set_buttons();

	LCD_Init(&hi2c1, LCD_ADDR);
#if CONTINUOUS_MODE && DIFFERENTIAL_MODE
	ads1115_init_continuous(ADS1115_ADDR, &hi2c1, 0, 1);  // AIN0-AIN1
#endif
#if CONTINUOUS_MODE && !DIFFERENTIAL_MODE
	ads1115_init_single_continuous(1); // Example
#endif
}

void routine(){
	update_temperatures(temp_channels, temp_channels_count);
	update_currents(current_channels, current_channels_count);
	update_voltages(voltage_channels, voltage_channels_count);
	update_buttons(buttons, buttons_count);
}



PowerChannel* get_max_temp(float* result){
	PowerChannel* ch = NULL;
	float max = -99.0f;
	for (uint8_t i = 0; i < temp_channels_count; i++){
		for (uint8_t j = 0; j < temp_channels_count; j++){
			if (temp_channels[i].temp_sensors[j].last_value > max){
				ch = &temp_channels[i];
				max = temp_channels[i].temp_sensors[j].last_value;
			}
		}
	}

	*result = max;
	return ch;
}

PowerChannel* get_max_current(float* result){
	PowerChannel* ch = NULL;
	float max = -99.0f;
	for (uint8_t i = 0; i < current_channels_count; i++){
		if (current_channels[i].current_sensor->last_value > max){
			ch = &current_channels[i];
			max = current_channels[i].current_sensor->last_value;
		}
	}

	*result = max;
	return ch;
}

PowerChannel* get_max_voltage(float* result){
	PowerChannel* ch = NULL;
	float max = -99.0f;
	for (uint8_t i = 0; i < voltage_channels_count; i++){
		if (voltage_channels[i].voltage_sensor->last_value > max){
			ch = &voltage_channels[i];
			max = voltage_channels[i].voltage_sensor->last_value;
		}
	}

	*result = max;
	return ch;
}

void set_warning_channel(int i){
	if (power_channels[i].in_warning_state){
		printf("Warning state found!\r\n");
		warning_channels_count += 1;
		warning_channels = realloc(warning_channels, warning_channels_count * sizeof(PowerChannel));
		warning_channels[warning_channels_count - 1] = power_channels[i];
	}
}

void set_shutdown_channel(int i){
	if (power_channels[i].in_shutdown_state){
		printf("Shutdown state found!\r\n");
		shutdown_channels_count += 1;
		shutdown_channels = realloc(shutdown_channels, shutdown_channels_count * sizeof(PowerChannel));
		shutdown_channels[shutdown_channels_count - 1] = power_channels[i];
	}
}

void set_alert_channels(){
	warning_channels_count = 0;
	shutdown_channels_count = 0;
	for (uint8_t i = 0; i < channels_count; i++){
		set_warning_channel(i);
		set_shutdown_channel(i);
	}
}

void delay(uint32_t ms){
	uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < ms){
		routine();
	}
}

void put_str(char* str, uint8_t str_len, char* temp, uint8_t temp_len, uint8_t* str_pos){
	for (uint8_t i = 0; i < temp_len; i++){
		str[*str_pos] = temp[i];
		(*str_pos)++;
		if (*str_pos >= str_len) return;
	}
}

void main_screen(){
	set_alert_channels();
	LCD_SetFirstLine(LCD_ADDR);
	char str[SCREEN_LENGTH];
	uint8_t str_pos = 0;
	for (uint8_t i = 0; i < channels_count; i++){
		put_str(str, 16, power_channels[i].name, strlen(power_channels[i].name), &str_pos);
		if (str_pos >= SCREEN_LENGTH) break;
		if (power_channels[i].enabled){
			str[str_pos] = 'O';
		} else {
			str[str_pos] = 'X';
		}
		str_pos++;
	}
	LCD_SendString(LCD_ADDR, str);
	LCD_SetSecondLine(LCD_ADDR);
	memset(str, 0, sizeof(str));
	str_pos = 0;
	if (shutdown_channels_count > 0){
		str[0] = '!';
		str_pos++;
		for (uint8_t i = 0; i < shutdown_channels_count; i++){
			put_str(str, 16, shutdown_channels[i].name, strlen(shutdown_channels[i].name), &str_pos);
			if (str_pos >= SCREEN_LENGTH) break;
		}
	} else if (warning_channels_count > 0){
		str[0] = '*';
		str_pos++;
		for (uint8_t i = 0; i < warning_channels_count; i++){
			put_str(str, 16, warning_channels[i].name, strlen(warning_channels[i].name), &str_pos);
			if (str_pos >= SCREEN_LENGTH) break;
		}
	} else {
		float value;
		char temp_str[5];
		PowerChannel* ch = get_max_temp(&value);
		put_str(str, 16, ch->name, strlen(ch->name), &str_pos);
		ftoa(value, temp_str, 1);
		put_str(str, 16, temp_str, strlen(temp_str), &str_pos);
		put_str(str, 16, "T ", 1, &str_pos);
		ch = get_max_current(&value);
		put_str(str, 16, ch->name, strlen(ch->name), &str_pos);
		ftoa(value, temp_str, 1);
		put_str(str, 16, temp_str, strlen(temp_str), &str_pos);
		put_str(str, 16, "A", 1, &str_pos);
	}
	LCD_SendString(LCD_ADDR, str);
}

void update_screen(){
	switch (state){
	case State_Main:
		main_screen();
		break;
	}
}

void main_loop(){
	routine();
	delay(1000);
	update_screen();
}



