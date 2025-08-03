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

#include "controller.h"
#include "defines.h"
#include "sensor_reader.h"
#include "project_types.h"
#include "ftoa.h"
#include "lcd_i2c.h"
#include "buttons.h"
#include "power_channel.h"
#include "custom_chars.h"


#define TEMP_DISPLAY_SIZE 3 // Maximum number of digits while displaying temperature

#define CURRENT_MAIN_SCREEN_PRECISION 2
#define CURRENT_DISPLAY_PRECISION 3 // Number of digits after '.'
#define CURRENT_DISPLAY_SIZE 2 + 1 + CURRENT_DISPLAY_PRECISION // 2 means there will be maximum of 2 numbers in integer part, 1 for '.'

#define VOLTAGE_DISPLAY_PRECISION 1 // Number of digits after '.'
#define VOLTAGE_DISPLAY_SIZE 2 + 1 + VOLTAGE_DISPLAY_PRECISION // 2 means there will be maximum of 2 numbers in integer part, 1 for '.'

#define SETTINGS_OPTIONS_COUNT 2


/** @brief Array of all power channels managed by the controller. */
static PowerChannel** power_channels = NULL;
static uint8_t channels_count = 0;
static Button** additional_buttons = NULL;
static uint8_t additional_buttons_count = 0;
static FanController** fan_controllers = NULL;
static uint8_t fan_controllers_count = 0;

/** @brief Pointers to channels that have specific sensor types, for efficient polling. */
static PowerChannel** temp_channels = NULL;
static uint8_t temp_channels_count = 0;
static PowerChannel** current_channels = NULL;
static uint8_t current_channels_count = 0;
static PowerChannel** voltage_channels = NULL;
static uint8_t voltage_channels_count = 0;
static PowerChannel** pwm_channels = NULL;
static uint8_t pwm_channels_count = 0;

/** @brief Pointers to channels that have an associated button. */
static PowerChannel** button_channels = NULL;
static uint8_t button_channels_count = 0;

/** @brief Master list of all buttons to be polled. */
static Button** buttons = NULL;
static uint8_t buttons_count = 0;

/** @brief Pointers to channels currently in a warning or shutdown state. */
static PowerChannel** warning_channels = NULL;
static uint8_t warning_channels_count = 0;
static PowerChannel** shutdown_channels = NULL;
static uint8_t shutdown_channels_count = 0;

/** @brief Represents the current screen being displayed on the LCD. */
typedef enum {
	State_Main,
	State_Channel,
	State_Settings
} ScreenState;
static ScreenState state = State_Main;
static uint8_t displayed_channel = 0;

/** @brief Represents the current screen of settings menu being displayed on the LCD. */
typedef enum {
	State_Settings_Main = 0,
	State_Settings_PWM
} ScreenStateSettings;
static char* settings_options[SETTINGS_OPTIONS_COUNT] = {"Main", "PWM"};
static ScreenStateSettings state_settings = State_Settings_Main;
static int8_t settings_pos = 1;
typedef enum {
	State_Settings_Menu_Channels = 0,
	State_Settings_Menu_Sensor,
	State_Settings_Menu_Settings
} ScreenStateSettingsMenu;
static ScreenStateSettingsMenu state_settings_menu = State_Settings_Menu_Channels;
static uint8_t state_settings_menu_channel = 0;
static uint8_t state_settings_menu_sensor = 0;

/** @brief Maps custom character symbols to their CGRAM locations. */
typedef enum {
    LCD_SYM_ON = 0,
    LCD_SYM_OFF,
	LCD_SYM_TEMP,
    LCD_SYM_WARNING,
    LCD_SYM_DANGER
} LcdSymbol;


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
static void init_custom_symbols(){
	LCD_CreateChar(LCD_ADDR, 0, symbol_on);
	LCD_CreateChar(LCD_ADDR, 1, symbol_off);
	LCD_CreateChar(LCD_ADDR, 2, symbol_temp);
	LCD_CreateChar(LCD_ADDR, 3, symbol_warning);
	LCD_CreateChar(LCD_ADDR, 4, symbol_danger);
}

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
	LCD_Init(hi2c, LCD_ADDR);
	init_custom_symbols();
	LCD_Clear(LCD_ADDR);

    // Initialize ADC subsystem (internal and/or external)
	init_sensors(hadc, hi2c);
	debug_printf("Controller initialization finished\r\n");
}

static void normal_behaviour(Button* button, uint8_t index, bool is_channel_button){
	switch (button->event) {
		case BUTTON_SHORT_PRESS:
			if (is_channel_button) {
				toggle_channel(button_channels[index]);
			} else if (index == 0 && state != State_Settings) {
				state = State_Settings;
				state_settings = State_Settings_Main;
			}
			break;
		case BUTTON_LONG_PRESS:
			if (is_channel_button && (state == State_Main || state == State_Channel)) {
				if (displayed_channel == index && state == State_Channel) {
					state = State_Main;
				} else {
					state = State_Channel;
					displayed_channel = index;
				}
			} else if (!is_channel_button && state == State_Settings) {
				if (index == 0) state = State_Main;
			}
			break;

		default:
			break;
	}
	button->event = BUTTON_IDLE;
}

/**
 * @brief Adjusts a specific digit of PWM percentage up or down in a "carousel" style.
 *        Handles boundary conditions (0–100%) smartly.
 * @param channel_idx Index of the PWM channel.
 * @param active_digit 0 - units, 1 - tens, 2 - hundreds.
 * @param increase true to increase, false to decrease.
 */
void pwm_carousel_adjust_digit(uint8_t channel_idx, uint8_t active_digit, bool increase) {
    OutputControl* output = &pwm_channels[channel_idx]->output;
    uint32_t period = output->pwm_timer->Init.Period;

    // Calculate current percentage
    float percent = output->pwm_inversed ?
                    100.0f - 100.0f * output->pwm_last_value / period :
                    100.0f * output->pwm_last_value / period;

    uint8_t perc_int = (uint8_t)(percent + 0.5f);

    // Determine the step size: 1, 10, or 100
    uint8_t step = 1;
    if (active_digit == 1) step = 10;
    else if (active_digit == 2) step = 100;

    // Adjust value
    if (increase) {
        if (perc_int + step <= 100)
            perc_int += step;
        else
            perc_int = (perc_int / step) * step; // "Wrap" to 0 for this digit
    } else {
        if (perc_int >= step)
            perc_int -= step;
        else
            perc_int = ((perc_int / step) * step) + 9 * step <= 100
                       ? ((perc_int / step) * step) + 9 * step
                       : 100;  // Clamp if wrap exceeds 100
    }

    // Convert to raw PWM
    uint32_t new_pwm = output->pwm_inversed ?
                       (uint32_t)((100.0f - perc_int) * period / 100.0f) :
                       (uint32_t)(perc_int * period / 100.0f);

    output->pwm_last_value = new_pwm;

    // Apply new PWM
    activate_channel(pwm_channels[channel_idx]);
}

void settings_behaviour(Button* button, uint8_t index, bool is_channel_button){
	switch (button->event) {
		case BUTTON_SHORT_PRESS:
			if (state_settings != State_Settings_Main && state_settings_menu == State_Settings_Menu_Channels){
				if (state_settings == State_Settings_PWM) state_settings_menu = State_Settings_Menu_Settings;
				else state_settings_menu = State_Settings_Menu_Sensor;
				state_settings_menu_channel = settings_pos;
			} else if (is_channel_button && index == 0 && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings){
				pwm_carousel_adjust_digit(state_settings_menu_channel, 2 - settings_pos, false);
			} else if (is_channel_button && index == 1 && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings){
				pwm_carousel_adjust_digit(state_settings_menu_channel, 2 - settings_pos, true);
			} else if (!is_channel_button && index == 0 && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings){
				settings_pos++;
			} else if (is_channel_button && index == 0) {
				settings_pos--;
			} else if (is_channel_button && index == 1) {
				settings_pos++;
			} else if (state_settings == State_Settings_Main && !is_channel_button && index == 0){
				state_settings = settings_pos;
			}
			break;
		case BUTTON_LONG_PRESS:
			if (!is_channel_button && index == 0) {
				if (state_settings == State_Settings_Main) state = State_Main;
				else {
					state_settings = State_Settings_Main;
					state_settings_menu = State_Settings_Menu_Channels;
				}
			}
			break;
		default:
			break;
	}
	button->event = BUTTON_IDLE;
}

/**
 * @brief Handles a detected button event based on the current screen state.
 * @param button Pointer to the button that generated the event.
 * @param index The index of the button/channel.
 * @param is_channel_button True if the button is tied to a power channel.
 */
static void handle_button_event(Button* button, uint8_t index, bool is_channel_button) {
	switch (state) {
		case State_Settings:
			settings_behaviour(button, index, is_channel_button);
			break;
		case State_Main:
		case State_Channel:
		default:
			normal_behaviour(button, index, is_channel_button);
			break;
	}
}

/**
 * @brief Checks all buttons for events and dispatches them to the handler.
 */
static void buttons_action() {
	for (uint8_t i = 0; i < button_channels_count; i++) {
		handle_button_event(button_channels[i]->button, i, true);
	}
	for (uint8_t i = 0; i < additional_buttons_count; i++) {
		handle_button_event(additional_buttons[i], i, false);
	}
}

/**
 * @brief Performs all periodic background tasks.
 * @details This function is the heart of the non-blocking operation. It updates all
 *          sensors and buttons.
 */
static void routine(){
	update_temperatures(temp_channels, temp_channels_count);
	update_currents(current_channels, current_channels_count);
	update_voltages(voltage_channels, voltage_channels_count);
	update_buttons(buttons, buttons_count);
	buttons_action();
}

/**
 * @brief Gets the maximum temperature and its "danger ratio" for a given channel.
 * @param ch The channel to check.
 * @param[out] value Pointer to store the highest temperature value found.
 * @return The ratio of the highest temperature to its shutdown threshold.
 */
static float get_max_temp_by_channel(PowerChannel* ch, int8_t *value){
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
static void set_warning_channel(int i){
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
static void set_shutdown_channel(int i){
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

void delay(uint32_t ms){
	uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < ms){
		routine();
	}
}

static uint8_t put_str(char* str, uint8_t str_len, char* temp, uint8_t temp_len, uint8_t str_pos){
	uint8_t added = 0;
	for (uint8_t i = 0; i < temp_len; i++){
		str[str_pos] = temp[i];
		str_pos++;
		added++;
		if (str_pos >= str_len) return added;
	}

	return added;
}

static void send_str(char* str){
	LCD_SendString(LCD_ADDR, str);
	memset(str, 0, SCREEN_LENGTH);
}

static bool print_channel(char* str, uint8_t *str_pos, PowerChannel *channel){
	(*str_pos) += put_str(str, SCREEN_LENGTH, channel->name, strlen(channel->name), 0);
	send_str(str);
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
		(*str_pos) += put_str(str, SCREEN_LENGTH, shutdown_channels[i]->name, strlen(shutdown_channels[i]->name), 0);
		send_str(str);
		if ((*str_pos) >= SCREEN_LENGTH) break;
	}
}

static void print_warning_channels(char* str, uint8_t *str_pos){
	LCD_SendData(LCD_ADDR, LCD_SYM_WARNING);
	(*str_pos)++;
	for (uint8_t i = 0; i < warning_channels_count; i++){
		(*str_pos) += put_str(str, 16, warning_channels[i]->name, strlen(warning_channels[i]->name), 0);
		send_str(str);
		if ((*str_pos) >= SCREEN_LENGTH) break;
	}
}

static void add_name(char* str, uint8_t *str_pos, PowerChannel* ch){
	(*str_pos) += put_str(str, SCREEN_LENGTH, ch->name, strlen(ch->name), 0);
	(*str_pos) += put_str(str, SCREEN_LENGTH, " ", 1, strlen(ch->name));
}

static void add_float(char* str, uint8_t *str_pos, PowerChannel* ch, int8_t value, bool name, uint8_t precision){
	char temp_str[TEMP_DISPLAY_SIZE];
	ftoa(value, temp_str, precision);
	(*str_pos) += put_str(str, SCREEN_LENGTH, temp_str, strlen(temp_str), name ? strlen(ch->name) + 1 : 0);
	send_str(str);
}

static void print_reading(char* str, uint8_t *str_pos, PowerChannel* ch, float value, bool name, uint8_t precision, char symbol){
	if (name) add_name(str, str_pos, ch);
	add_float(str, str_pos, ch, value, name, precision);
	LCD_SendData(LCD_ADDR, symbol);
}

static void print_max_temp_current(char* str, uint8_t *str_pos){
	int8_t temp_value;
	PowerChannel* ch = get_max_temp(&temp_value);
	print_reading(str, str_pos, ch, temp_value, true, 0, LCD_SYM_TEMP);
	(*str_pos) = 0;
	LCD_SendString(LCD_ADDR, " ");
	float current_value;
	ch = get_max_current(&current_value);
	print_reading(str, str_pos, ch, current_value, true, CURRENT_MAIN_SCREEN_PRECISION, 'A');

}

static void clear_line_end(char *str){
	put_str(str, SCREEN_LENGTH, "                ", SCREEN_LENGTH, 0);
	send_str(str);
}

static void main_screen(){
	set_alert_channels();
	LCD_SetFirstLine(LCD_ADDR);
	char str[SCREEN_LENGTH];
	uint8_t str_pos = 0;
	print_channels(str, &str_pos);
	clear_line_end(str);
	LCD_SetSecondLine(LCD_ADDR);
	str_pos = 0;
	if (shutdown_channels_count > 0){
		print_danger_channels(str, &str_pos);
	} else if (warning_channels_count > 0){
		print_warning_channels(str, &str_pos);
	} else {
		print_max_temp_current(str, &str_pos);
	}
	clear_line_end(str);
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
		str_pos += put_str(str, SCREEN_LENGTH, "No A", 4, 0);
		send_str(str);
	}
	LCD_SendData(LCD_ADDR, ' ');
	if (power_channels[displayed_channel]->voltage_sensor != NULL){
		print_reading(str, &str_pos, power_channels[displayed_channel], power_channels[displayed_channel]->voltage_sensor->last_value, false, VOLTAGE_DISPLAY_PRECISION, 'V');
	} else {
		str_pos += put_str(str, SCREEN_LENGTH, "No V", 4, 0);
		send_str(str);
	}
	clear_line_end(str);
	LCD_SetSecondLine(LCD_ADDR);
	str_pos = 0;
	if (power_channels[displayed_channel]->temp_sensor_count > 0){
		int8_t temp;
		get_max_temp_by_channel(power_channels[displayed_channel], &temp);
		print_reading(str, &str_pos, power_channels[displayed_channel], temp, false, 0, LCD_SYM_TEMP);
	} else {
		str_pos += put_str(str, SCREEN_LENGTH, "No T", 4, 0);
		send_str(str);
	}
	LCD_SendData(LCD_ADDR, ' ');
	if (power_channels[displayed_channel]->output.type == OUTPUT_PWM){
		str_pos += put_str(str, SCREEN_LENGTH, "PWM ", 4, 0);
		send_str(str);
		float percentage = power_channels[displayed_channel]->output.pwm_inversed ?
				100 - 100 * power_channels[displayed_channel]->output.pwm_last_value / power_channels[displayed_channel]->output.pwm_timer->Init.Period :
				100 * power_channels[displayed_channel]->output.pwm_last_value / power_channels[displayed_channel]->output.pwm_timer->Init.Period;
		ftoa(percentage, str, 0);
		send_str(str);
		LCD_SendData(LCD_ADDR, '%');
	} else {
		str_pos += put_str(str, SCREEN_LENGTH, "GPIO ", 5, 0);
		send_str(str);
	}
	clear_line_end(str);
}

static void settings_screen(){
	char str[SCREEN_LENGTH];
	uint8_t str_pos = 0;
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
			str_pos += put_str(str, SCREEN_LENGTH, i == 0 ? ">" : "-", 1, 0);
			str_pos += put_str(str, SCREEN_LENGTH, settings_options[n], strlen(settings_options[n]), 1);
			send_str(str);
			clear_line_end(str);
		}
		break;
	case State_Settings_PWM:
		if (state_settings_menu == State_Settings_Menu_Channels){
			if (settings_pos >= pwm_channels_count || settings_pos < 0){
				settings_pos = 0;
			}
			for (int i = 0; i < 2; i++){
				if (i == 0){
					LCD_SetFirstLine(LCD_ADDR);
				} else {
					LCD_SetSecondLine(LCD_ADDR);
				}
				uint8_t n = (settings_pos + i) % pwm_channels_count;
				str_pos += put_str(str, SCREEN_LENGTH, i == 0 ? ">" : "-", 1, 0);
				str_pos += put_str(str, SCREEN_LENGTH, pwm_channels[n]->name, 3, 1);
				if (pwm_channels_count > 2 || i == 0) send_str(str);
				clear_line_end(str);
			}
		} else if (state_settings_menu == State_Settings_Menu_Settings){
			float percentage = pwm_channels[state_settings_menu_channel]->output.pwm_inversed ?
					100 - 100 * pwm_channels[state_settings_menu_channel]->output.pwm_last_value / pwm_channels[state_settings_menu_channel]->output.pwm_timer->Init.Period :
					100 * pwm_channels[state_settings_menu_channel]->output.pwm_last_value / pwm_channels[state_settings_menu_channel]->output.pwm_timer->Init.Period;
			char temp[3];
			ftoa(percentage, temp, 0);
			settings_pos %= 3;
			for (int i = 0; i < 2; i++){
				if (i == 0){
					LCD_SetFirstLine(LCD_ADDR);
					str_pos += put_str(str, SCREEN_LENGTH, pwm_channels[state_settings_menu_channel]->name, strlen(pwm_channels[state_settings_menu_channel]->name), 0);
					str_pos += put_str(str, SCREEN_LENGTH, "  ", 2, strlen(pwm_channels[state_settings_menu_channel]->name));
					send_str(str);
					put_str(str, SCREEN_LENGTH, "00", 2, 0);
					put_str(str, SCREEN_LENGTH, temp, strlen(temp), 3 - strlen(temp));
					send_str(str);
					LCD_SendData(LCD_ADDR, '%');
				} else {
					LCD_SetSecondLine(LCD_ADDR);
					for (int j = 0; j < strlen(pwm_channels[state_settings_menu_channel]->name) + 2 + settings_pos; j++){
						str[j] = ' ';
					}
					send_str(str);
					LCD_SendData(LCD_ADDR, '-');
				}
				clear_line_end(str);
			}
		}
		break;
	default:
		if (settings_pos >= SETTINGS_OPTIONS_COUNT || settings_pos < 1){
			settings_pos = 1;
		}
		for (int i = 0; i < 2; i++){
			if (i == 0){
				LCD_SetFirstLine(LCD_ADDR);
			} else {
				LCD_SetSecondLine(LCD_ADDR);
			}
			str_pos += put_str(str, SCREEN_LENGTH, i == 0 ? ">" : "-", 1, 0);
			str_pos += put_str(str, SCREEN_LENGTH, "Other", 5, 1);
			send_str(str);
			clear_line_end(str);
		}
		break;
	}
}

static void update_screen(){
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



void main_loop(){
	routine();
	delay(500);
	update_screen();
}



