/*
 * controller_globals.c
 *
 *  Created on: Aug 3, 2025
 *      Author: kiril
 */

#include <inttypes.h>
#include <stdbool.h>

#include "controller_globals.h"
#include "project_types.h"

/** @brief Indicates whether the screen should be updated immediately. */
bool refresh_screen = false;

/** @brief Array of all power channels managed by the controller. */
PowerChannel** power_channels = NULL;
uint8_t channels_count = 0;
Button** additional_buttons = NULL;
uint8_t additional_buttons_count = 0;
FanController** fan_controllers = NULL;
uint8_t fan_controllers_count = 0;

/** @brief Pointers to channels that have specific sensor types, for efficient polling. */
PowerChannel** temp_channels = NULL;
uint8_t temp_channels_count = 0;
PowerChannel** current_channels = NULL;
uint8_t current_channels_count = 0;
PowerChannel** voltage_channels = NULL;
uint8_t voltage_channels_count = 0;
PowerChannel** pwm_channels = NULL;
uint8_t pwm_channels_count = 0;

/** @brief Pointers to channels that have an associated button. */
PowerChannel** button_channels = NULL;
uint8_t button_channels_count = 0;

/** @brief Master list of all buttons to be polled. */
Button** buttons = NULL;
uint8_t buttons_count = 0;

/** @brief Pointers to channels currently in a warning or shutdown state. */
PowerChannel** warning_channels = NULL;
uint8_t warning_channels_count = 0;
PowerChannel** shutdown_channels = NULL;
uint8_t shutdown_channels_count = 0;

ScreenState state = State_Main;
uint8_t displayed_channel = 0;

char* settings_options[SETTINGS_OPTIONS_COUNT] = {"Main", "PWM", "Current"};
ScreenStateSettings state_settings = State_Settings_Main;
int8_t settings_pos = 1;
int8_t settings_pos2 = 0;

ScreenStateSettingsMenu state_settings_menu = State_Settings_Menu_Channels;
uint8_t state_settings_menu_channel = 0;
uint8_t state_settings_menu_sensor = 0;

