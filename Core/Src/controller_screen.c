/*
 * controller_screen.c
 *
 *  Created on: Aug 3, 2025
 *      Author: kiril
 */
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

#include "controller_globals.h"
#include "controller_screen.h"
#include "project_types.h"
#include "lcd_i2c.h"
#include "custom_chars.h"
#include "defines.h"
#include "ftoa.h"


typedef enum {
    LCD_SYM_ON = 0,
    LCD_SYM_OFF,
	LCD_SYM_TEMP,
    LCD_SYM_WARNING,
    LCD_SYM_DANGER
} LcdSymbol;

static void init_custom_symbols(){
	LCD_CreateChar(LCD_ADDR, 0, symbol_on);
	LCD_CreateChar(LCD_ADDR, 1, symbol_off);
	LCD_CreateChar(LCD_ADDR, 2, symbol_temp);
	LCD_CreateChar(LCD_ADDR, 3, symbol_warning);
	LCD_CreateChar(LCD_ADDR, 4, symbol_danger);
}

void controller_screen_init(I2C_HandleTypeDef *hi2c){
	LCD_Init(hi2c, LCD_ADDR);
	init_custom_symbols();
	LCD_Clear(LCD_ADDR);
}

/**
 * @brief Gets the maximum temperature and its "danger ratio" for a given channel.
 * @param ch The channel to check.
 * @param[out] value Pointer to store the highest temperature value found.
 * @return The ratio of the highest temperature to its shutdown threshold.
 */
static float get_max_temp_by_channel(const PowerChannel* ch, int8_t *value){
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
static PowerChannel* get_max_temp(int8_t* result){
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
static PowerChannel* get_max_current(float* result){
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
static PowerChannel* get_max_voltage(float* result){
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
static void set_alert_channels(){
	warning_channels_count = 0;
	shutdown_channels_count = 0;
	for (uint8_t i = 0; i < channels_count; i++){
		set_warning_channel(i);
		set_shutdown_channel(i);
	}
}

static uint8_t put_str(char* dest, const char* src, uint8_t start_pos, uint8_t max_len) {
	uint8_t added = 0;
	for (uint8_t i = 0; i < strlen(src); i++){
		dest[start_pos] = src[i];
		start_pos++;
		added++;
		if (start_pos >= max_len) return added;
	}

	return added;
}

static void send_str(char* str, bool clear){
	LCD_SendString(LCD_ADDR, str);
	if (clear) memset(str, 0, SCREEN_LENGTH);
}

static bool print_channel(char* str, uint8_t *str_pos, PowerChannel *channel){
	(*str_pos) += put_str(str, channel->name, 0, SCREEN_LENGTH);
	send_str(str, true);
	if ((*str_pos) >= SCREEN_LENGTH) return true;
	if (channel->enabled){
		LCD_SendData(LCD_ADDR, LCD_SYM_ON);
	} else {
		LCD_SendData(LCD_ADDR, LCD_SYM_OFF);
	}
	(*str_pos)++;

	return false;
}

static void print_channels(char* str, uint8_t *str_pos){
	for (uint8_t i = 0; i < channels_count; i++){
		if (print_channel(str, str_pos, power_channels[i])) break;
	}
}

static void print_danger_channels(char* str, uint8_t *str_pos){
	LCD_SendData(LCD_ADDR, LCD_SYM_DANGER);
	(*str_pos)++;
	for (uint8_t i = 0; i < shutdown_channels_count; i++){
		(*str_pos) += put_str(str, shutdown_channels[i]->name, 0, SCREEN_LENGTH);
		send_str(str, true);
		if ((*str_pos) >= SCREEN_LENGTH) break;
	}
}

static void print_warning_channels(char* str, uint8_t *str_pos){
	LCD_SendData(LCD_ADDR, LCD_SYM_WARNING);
	(*str_pos)++;
	for (uint8_t i = 0; i < warning_channels_count; i++){
		(*str_pos) += put_str(str,  warning_channels[i]->name, 0, SCREEN_LENGTH);
		send_str(str, true);
		if ((*str_pos) >= SCREEN_LENGTH) break;
	}
}

static void add_name(char* str, uint8_t *str_pos, const PowerChannel* ch){
	(*str_pos) += put_str(str, ch->name, 0, SCREEN_LENGTH);
	(*str_pos) += put_str(str, " ", strlen(ch->name), SCREEN_LENGTH);
}

static void add_float(char* str, uint8_t *str_pos, const PowerChannel* ch, float value, bool name, uint8_t precision){
	char temp_str[TEMP_DISPLAY_SIZE];
	ftoa(value, temp_str, precision);
	(*str_pos) += put_str(str, temp_str, name ? strlen(ch->name) + 1 : 0, SCREEN_LENGTH);
	send_str(str, true);
}

static void print_reading(char* str, uint8_t *str_pos, const PowerChannel* ch, float value, bool name, uint8_t precision, char symbol){
	if (name) add_name(str, str_pos, ch);
	add_float(str, str_pos, ch, value, name, precision);
	LCD_SendData(LCD_ADDR, symbol);
}

static void print_max_temp_current(char* str, uint8_t *str_pos){
	int8_t temp_value;
	PowerChannel* ch = get_max_temp(&temp_value);
	print_reading(str, str_pos, ch, temp_value, true, 0, LCD_SYM_TEMP);
	LCD_SendString(LCD_ADDR, " ");
	float current_value;
	ch = get_max_current(&current_value);
	print_reading(str, str_pos, ch, current_value, true, CURRENT_MAIN_SCREEN_PRECISION, 'A');

}

static void clear_line_end(){
	char str[SCREEN_LENGTH];
	put_str(str, "                ", 0, SCREEN_LENGTH);
	send_str(str, true);
}

static void main_screen(){
	set_alert_channels();
	LCD_SetFirstLine(LCD_ADDR);
	char str[SCREEN_LENGTH];
	uint8_t str_pos = 0;
	print_channels(str, &str_pos);
	clear_line_end();
	LCD_SetSecondLine(LCD_ADDR);
	str_pos = 0;
	if (shutdown_channels_count > 0){
		print_danger_channels(str, &str_pos);
	} else if (warning_channels_count > 0){
		print_warning_channels(str, &str_pos);
	} else {
		print_max_temp_current(str, &str_pos);
	}
	clear_line_end();
}

static void channel_screen(){
	char str[SCREEN_LENGTH];
	uint8_t str_pos = 0;
	LCD_SetFirstLine(LCD_ADDR);
	print_channel(str, &str_pos, power_channels[displayed_channel]);
	LCD_SendData(LCD_ADDR, ' ');
	if (power_channels[displayed_channel]->current_sensor != NULL){
		print_reading(str, &str_pos, power_channels[displayed_channel], power_channels[displayed_channel]->current_sensor->last_value, false, CURRENT_DISPLAY_PRECISION, 'A');
	} else {
		send_str("No A", false);
	}
	LCD_SendData(LCD_ADDR, ' ');
	if (power_channels[displayed_channel]->voltage_sensor != NULL){
		print_reading(str, &str_pos, power_channels[displayed_channel], power_channels[displayed_channel]->voltage_sensor->last_value, false, VOLTAGE_DISPLAY_PRECISION, 'V');
	} else {
		send_str("No V", false);
	}
	clear_line_end();
	LCD_SetSecondLine(LCD_ADDR);
	if (power_channels[displayed_channel]->temp_sensor_count > 0){
		int8_t temp;
		get_max_temp_by_channel(power_channels[displayed_channel], &temp);
		print_reading(str, &str_pos, power_channels[displayed_channel], temp, false, 0, LCD_SYM_TEMP);
	} else {
		send_str("No T", false);
	}
	LCD_SendData(LCD_ADDR, ' ');
	if (power_channels[displayed_channel]->output.type == OUTPUT_PWM){
		send_str("PWM ", false);
		float percentage = power_channels[displayed_channel]->output.pwm_inversed ?
				100 - 100 * power_channels[displayed_channel]->output.pwm_last_value / power_channels[displayed_channel]->output.pwm_timer->Init.Period :
				100 * power_channels[displayed_channel]->output.pwm_last_value / power_channels[displayed_channel]->output.pwm_timer->Init.Period;
		ftoa(percentage, str, 0);
		send_str(str, true);
		LCD_SendData(LCD_ADDR, '%');
	} else {
		send_str("GPIO ", false);
	}
	clear_line_end();
}

static void settings_screen(){
	switch (state_settings){
	case State_Settings_Main:
		if (settings_pos >= SETTINGS_OPTIONS_COUNT || settings_pos < 1){
			settings_pos = 1;
		}
		for (int i = 0; i < 2; i++){
			if (i == 0){
				LCD_SetFirstLine(LCD_ADDR);
			} else {
				LCD_SetSecondLine(LCD_ADDR);
			}
			uint8_t n = settings_pos + i < SETTINGS_OPTIONS_COUNT ? settings_pos + i : 1;
			send_str(i == 0 ? ">" : "-", false);
			send_str(settings_options[n], false);
			clear_line_end();
		}
		break;
	case State_Settings_PWM:
		if (state_settings_menu == State_Settings_Menu_Channels){
			if (settings_pos >= pwm_channels_count || settings_pos < 0){
				settings_pos = 0;
			}
			for (uint8_t i = 0; i < 2; i++){
				if (i == 0){
					LCD_SetFirstLine(LCD_ADDR);
				} else {
					LCD_SetSecondLine(LCD_ADDR);
				}
				uint8_t n = (settings_pos + i) % pwm_channels_count;
				if (pwm_channels_count > 2 || i == 0){

					send_str(i == 0 ? ">" : "-", false);
					send_str(pwm_channels[n]->name, false);
				}
				clear_line_end();
			}
		} else if (state_settings_menu == State_Settings_Menu_Settings){
			float percentage = pwm_channels[state_settings_menu_channel]->output.pwm_inversed ?
					100 - 100 * pwm_channels[state_settings_menu_channel]->output.pwm_last_value / pwm_channels[state_settings_menu_channel]->output.pwm_timer->Init.Period :
					100 * pwm_channels[state_settings_menu_channel]->output.pwm_last_value / pwm_channels[state_settings_menu_channel]->output.pwm_timer->Init.Period;
			char temp[3];
			ftoa(percentage, temp, 0);
			settings_pos %= 3;
			for (uint8_t i = 0; i < 2; i++){
				if (i == 0){
					LCD_SetFirstLine(LCD_ADDR);
					send_str(pwm_channels[state_settings_menu_channel]->name, false);
					send_str("  ", false);
					for (uint8_t j = 0; j < 3 - strlen(temp); j++){
						send_str("0", false);
					}
					send_str(temp, false);
					LCD_SendData(LCD_ADDR, '%');
				} else {
					LCD_SetSecondLine(LCD_ADDR);
					for (int j = 0; j < strlen(pwm_channels[state_settings_menu_channel]->name) + 2 + settings_pos; j++){
						send_str(" ", false);
					}
					LCD_SendData(LCD_ADDR, '-');
				}
				clear_line_end();
			}
		}
		break;
	default:
		if (settings_pos >= SETTINGS_OPTIONS_COUNT || settings_pos < 1){
			settings_pos = 1;
		}
		for (uint8_t i = 0; i < 2; i++){
			if (i == 0){
				LCD_SetFirstLine(LCD_ADDR);
			} else {
				LCD_SetSecondLine(LCD_ADDR);
			}
			send_str(i == 0 ? ">" : "-", false);
			send_str("Other", false);
			clear_line_end();
		}
		break;
	}
}

void update_screen(){
	switch (state){
	case State_Main:
		main_screen();
		break;
	case State_Channel:
		channel_screen();
		break;
	case State_Settings:
		settings_screen();
		break;
	}
}
