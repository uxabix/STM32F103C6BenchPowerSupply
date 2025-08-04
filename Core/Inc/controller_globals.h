/*
 * controller_globals.h
 *
 *  Created on: Aug 3, 2025
 *      Author: kiril
 */

#ifndef INC_CONTROLLER_GLOBALS_H_
#define INC_CONTROLLER_GLOBALS_H_

#include <inttypes.h>
#include <stdbool.h>

#include "project_types.h"

#define SCREEN_UPDATE_DELAY 500 // Time between each screen update
#define TEMP_DISPLAY_SIZE 3 // Maximum number of digits while displaying temperature

#define CURRENT_MAIN_SCREEN_PRECISION 2
#define CURRENT_DISPLAY_PRECISION 3 // Number of digits after '.'
#define CURRENT_DISPLAY_SIZE 2 + 1 + CURRENT_DISPLAY_PRECISION // 2 means there will be maximum of 2 numbers in integer part, 1 for '.'
#define MAX_CURRENT_THRESHOLD 19.99f
#define MIN_CURRENT_THRESHOLD 0.005f

#define VOLTAGE_DISPLAY_PRECISION 1 // Number of digits after '.'
#define VOLTAGE_DISPLAY_SIZE 2 + 1 + VOLTAGE_DISPLAY_PRECISION // 2 means there will be maximum of 2 numbers in integer part, 1 for '.'

#define SETTINGS_OPTIONS_COUNT 3

#define SETTINGS_BUTTON !is_channel_button && index == 0
#define SETTINGS_BUTTON_Decrease is_channel_button && index == 0
#define SETTINGS_BUTTON_Increase is_channel_button && index == 1

/** @brief Indicates whether the screen should be updated immediately. */
extern bool refresh_screen;

/** @brief Array of all power channels managed by the controller. */
extern PowerChannel** power_channels;
extern uint8_t channels_count;
extern Button** additional_buttons;
extern uint8_t additional_buttons_count;
extern FanController** fan_controllers;
extern uint8_t fan_controllers_count;

/** @brief Pointers to channels that have specific sensor types, for efficient polling. */
extern PowerChannel** temp_channels;
extern uint8_t temp_channels_count;
extern PowerChannel** current_channels;
extern uint8_t current_channels_count;
extern PowerChannel** voltage_channels;
extern uint8_t voltage_channels_count;
extern PowerChannel** pwm_channels;
extern uint8_t pwm_channels_count;

/** @brief Pointers to channels that have an associated button. */
extern PowerChannel** button_channels;
extern uint8_t button_channels_count;

/** @brief Master list of all buttons to be polled. */
extern Button** buttons;
extern uint8_t buttons_count;

/** @brief Pointers to channels currently in a warning or shutdown state. */
extern PowerChannel** warning_channels;
extern uint8_t warning_channels_count;
extern PowerChannel** shutdown_channels;
extern uint8_t shutdown_channels_count;

/** @brief Represents the current screen being displayed on the LCD. */
typedef enum {
	State_Main,
	State_Channel,
	State_Settings
} ScreenState;
extern ScreenState state;
extern uint8_t displayed_channel;

/** @brief Represents the current screen of settings menu being displayed on the LCD. */
typedef enum {
	State_Settings_Main = 0,
	State_Settings_PWM,
	State_Settings_Current
} ScreenStateSettings;
extern char* settings_options[SETTINGS_OPTIONS_COUNT];
extern ScreenStateSettings state_settings;
extern int8_t settings_pos;
extern int8_t settings_pos2;

typedef enum {
	State_Settings_Menu_Channels = 0,
	State_Settings_Menu_Sensor,
	State_Settings_Menu_Settings
} ScreenStateSettingsMenu;
extern ScreenStateSettingsMenu state_settings_menu;
extern uint8_t state_settings_menu_channel;
extern uint8_t state_settings_menu_sensor;

#endif /* INC_CONTROLLER_GLOBALS_H_ */
