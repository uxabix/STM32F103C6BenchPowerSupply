/*
 * controller.c
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#include <stddef.h>

#include "project_types.h"

PowerChannel* channels = NULL;
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

bool add_temp_channels(uint8_t i){
	if (channels[i].temp_sensor_count > 0){
		printf("Has temp!\r\n");
		temp_channels_count += 1;
		temp_channels = realloc(temp_channels, temp_channels_count * sizeof(PowerChannel));
		temp_channels[temp_channels_count - 1] = channels[i];
	}
}

bool add_current_channels(uint8_t i){
	if (channels[i].current_sensor != NULL){
		printf("Has current!\r\n");
		current_channels_count += 1;
		current_channels = realloc(current_channels, current_channels_count * sizeof(PowerChannel));
		current_channels[current_channels_count - 1] = channels[i];
	}
}

bool add_voltage_channels(uint8_t i){
	if (channels[i].voltage_sensor != NULL){
		printf("Has voltage!\r\n");
		voltage_channels_count += 1;
		voltage_channels = realloc(voltage_channels, voltage_channels_count * sizeof(PowerChannel));
		voltage_channels[voltage_channels_count - 1] = channels[i];
	}
}

bool add_buttons_channels(uint8_t i){
	if (channels[i].button != NULL){
		printf("Has button!\r\n");
		button_channels_count += 1;
		button_channels = realloc(button_channels, button_channels_count * sizeof(PowerChannel));
		button_channels[button_channels_count - 1] = channels[i];
	}
}

void init_pwm_channel(OutputControl* pwm_channel){
	if (channels[i].output.type == OUTPUT_PWM){
		HAL_TIM_PWM_Start(pwm_channel->pwm_timer, pwm_channel->pwm_channel);
	}
}

bool set_buttons(){
	for (uint8_t i = 0; i < button_channels_count; i++){
		printf("New button!\r\n");
		buttons_count += 1;
		buttons = realloc(buttons, buttons_count * sizeof(PowerChannel));
		buttons[buttons_count - 1] = channels[i].button;
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
	channels = ch;
	channels_count = count;
	additional_buttons = buttons;
	additional_buttons_count = btn_count;
	fan_controllers = fans;

	for (uint8_t i = 0; i < channels_count; i++){
		add_temp_channels(i);
		add_current_channels(i);
		add_voltage_channels(i);
		add_buttons_channels(i);
		init_pwm_channel(channels[i].output);
	}
	set_buttons();

	LCD_Init(&hi2c1, LCD_ADDR);
#if CONTINUOUS_MODE && NON_DIFFERENTIAL_MODE == false
	ads1115_init_continuous(ADS1115_ADDR, &hi2c1, 0, 1);  // AIN0-AIN1
#if CONTINUOUS_MODE && NON_DIFFERENTIAL_MODE
	ads1115_init_single_continuous(1); // Example
#endif
}

void routine(){

}

void delay(uint32_t ms){

}



