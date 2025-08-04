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
#include "controller_getset.h"
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


/**
 * @brief Loads the custom character bitmaps into the LCD's CGRAM.
 */
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

static char get_status_current(const PowerChannel* channel){
	char res = '\0';
	if (channel->current_sensor == NULL)
		return res;
	if (channel->current_sensor->warning_triggered)
		res = LCD_SYM_WARNING;
	if (channel->current_sensor->shutdown_triggered)
		res = LCD_SYM_DANGER;

	return res;
}

static char get_status_voltage(const PowerChannel* channel){
	char res = '\0';
	if (channel->voltage_sensor == NULL)
		return res;
	if (channel->voltage_sensor->undervoltage_triggered)
		res = LCD_SYM_WARNING;
	if (channel->voltage_sensor->overvoltage_triggered)
		res = LCD_SYM_DANGER;

	return res;
}

static char get_status_temp(const PowerChannel* channel){
	char res = '\0';
	if (!channel->in_shutdown_state && !channel->in_warning_state)
		return res;
	for (uint8_t i = 0; i < channel->temp_sensor_count; i++){
		if (res == '\0' && channel->temp_sensors[i].warning_triggered)
			res = LCD_SYM_WARNING;
		if (channel->temp_sensors[i].shutdown_triggered)
			return LCD_SYM_DANGER;
	}
	return res;
}

static void channel_screen(){
	char str[SCREEN_LENGTH];
	uint8_t str_pos = 0;
	LCD_SetFirstLine(LCD_ADDR);
	print_channel(str, &str_pos, power_channels[displayed_channel]);
	LCD_SendData(LCD_ADDR, ' ');

	if (power_channels[displayed_channel]->current_sensor != NULL){
		char status = get_status_current(power_channels[displayed_channel]);
		if (status != '\0') LCD_SendData(LCD_ADDR, status);
		print_reading(str, &str_pos, power_channels[displayed_channel], power_channels[displayed_channel]->current_sensor->last_value, false, CURRENT_DISPLAY_PRECISION, 'A');
	} else {
		send_str("No A", false);
	}
	LCD_SendData(LCD_ADDR, ' ');
	if (power_channels[displayed_channel]->voltage_sensor != NULL){
		char status = get_status_voltage(power_channels[displayed_channel]);
		if (status != '\0') LCD_SendData(LCD_ADDR, status);
		print_reading(str, &str_pos, power_channels[displayed_channel], power_channels[displayed_channel]->voltage_sensor->last_value, false, VOLTAGE_DISPLAY_PRECISION, 'V');
	} else {
		send_str("No V", false);
	}
	clear_line_end();
	LCD_SetSecondLine(LCD_ADDR);
	if (power_channels[displayed_channel]->temp_sensor_count > 0){
		char status = get_status_temp(power_channels[displayed_channel]);
		if (status != '\0') LCD_SendData(LCD_ADDR, status);
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
