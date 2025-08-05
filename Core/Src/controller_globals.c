/**
 * @file controller_globals.c
 * @brief Global state variables for the controller logic.
 *
 * This file contains all global data structures shared across modules,
 * such as lists of power channels, sensor subsets, button mappings, screen states,
 * and settings UI selections.
 *
 * @author kiril
 * @date August 3, 2025
 */

#include <inttypes.h>
#include <stdbool.h>

#include "controller_globals.h"
#include "project_types.h"

/** @brief Indicates whether the screen should be updated immediately. */
bool refresh_screen = false;

/** @brief Array of all power channels managed by the controller. */
PowerChannel** power_channels = NULL;
/** @brief Total number of power channels. */
uint8_t channels_count = 0;
/** @brief Additional buttons not tied to specific channels (e.g., global menu/back buttons). */
Button** additional_buttons = NULL;
/** @brief Number of additional standalone buttons. */
uint8_t additional_buttons_count = 0;
/** @brief Array of all fan controllers in the system. */
FanController** fan_controllers = NULL;
/** @brief Number of fan controllers. */
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

/** @brief Current active screen state (e.g., main view, settings, alerts). */
ScreenState state = State_Main;
/** @brief Index of the channel currently being displayed. */
uint8_t displayed_channel = 0;

/** @brief String representations of the main settings categories. */
char* settings_options[SETTINGS_OPTIONS_COUNT] = {"Main", "PWM", "Current"};
/** @brief The current high-level settings screen being displayed. */
ScreenStateSettings state_settings = State_Settings_Main;
/** @brief The primary cursor position or selection index within a settings screen. */
int8_t settings_pos = 1;
/** @brief A secondary cursor or state, used for multi-level adjustments (e.g., choosing between warning/shutdown threshold). */
int8_t settings_pos2 = 0;

/** @brief The navigation depth within a specific setting's configuration menu. */
ScreenStateSettingsMenu state_settings_menu = State_Settings_Menu_Channels;
/** @brief The channel index currently being configured in the settings menu. */
uint8_t state_settings_menu_channel = 0;
/** @brief The sensor index currently being configured (used if a channel has multiple sensors of the same type). */
uint8_t state_settings_menu_sensor = 0;

