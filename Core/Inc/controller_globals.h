/**
 * @file controller_globals.h
 * @brief Global variables, constants, and screen state definitions for the controller logic.
 *
 * This header defines all global variables and screen/menu state structures
 * used across controller modules, including power channels, sensors,
 * buttons, fan controllers, and UI settings navigation.
 *
 * @author kiril
 * @date August 3, 2025
 */


#ifndef INC_CONTROLLER_GLOBALS_H_
#define INC_CONTROLLER_GLOBALS_H_

#include <inttypes.h>
#include <stdbool.h>

#include "project_types.h"

/** @brief Delay in milliseconds between automatic screen updates. */
#define SCREEN_UPDATE_DELAY 500
/** @brief Maximum number of digits for temperature display (e.g., "125"). */
#define TEMP_DISPLAY_SIZE 3

/** @brief Number of decimal places for current on main screen. */
#define CURRENT_MAIN_SCREEN_PRECISION 2
/** @brief Number of decimal places for current display. */
#define CURRENT_DISPLAY_PRECISION 3
/** @brief Max length of current string representation (e.g., "19.999"). */
#define CURRENT_DISPLAY_SIZE 2 + 1 + CURRENT_DISPLAY_PRECISION
/** @brief Upper limit for current threshold values. */
#define MAX_CURRENT_THRESHOLD 19.99f
/** @brief Lower limit for current threshold values. */
#define MIN_CURRENT_THRESHOLD 0.005f

#define VOLTAGE_DISPLAY_PRECISION 1 // Number of digits after '.'
#define VOLTAGE_DISPLAY_SIZE 2 + 1 + VOLTAGE_DISPLAY_PRECISION // 2 means there will be maximum of 2 numbers in integer part, 1 for '.'

#define SETTINGS_OPTIONS_COUNT 3

/**
 * @brief Macro to check if the main settings button is pressed (non-channel, index 0).
 * @param is_channel_button True if the button is tied to a channel.
 * @param index Index of the button being evaluated.
 * @return True if it's the main settings button.
 */
#define SETTINGS_BUTTON !is_channel_button && index == 0
/** @brief Macro to identify the 'Decrease' button in settings (channel button at index 0). */
#define SETTINGS_BUTTON_Decrease is_channel_button && index == 0
/** @brief Macro to identify the 'Increase' button in settings (channel button at index 1). */
#define SETTINGS_BUTTON_Increase is_channel_button && index == 1

/** @brief Indicates whether the screen should be updated immediately. */
extern bool refresh_screen;

/** @brief Array of all power channels managed by the controller. */
extern PowerChannel** power_channels;
/** @brief Total number of power channels. */
extern uint8_t channels_count;
/** @brief Additional buttons not tied to specific channels (e.g., global menu/back buttons). */
extern Button** additional_buttons;
/** @brief Number of additional standalone buttons. */
extern uint8_t additional_buttons_count;
/** @brief Array of all fan controllers in the system. */
extern FanController** fan_controllers;
/** @brief Number of fan controllers. */
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
	State_Main,     /**< Default main display screen. */
	State_Channel,  /**< Detailed channel view. */
	State_Settings, /**< Settings screen. */
} ScreenState;
/** @brief Current active screen state (e.g., main view, settings, alerts). */
extern ScreenState state;
/** @brief Index of the channel currently being displayed (in multi-channel mode). */
extern uint8_t displayed_channel;

/** @brief Represents the current screen of settings menu being displayed on the LCD. */
typedef enum {
	State_Settings_Main = 0,
	State_Settings_PWM,
	State_Settings_Current
} ScreenStateSettings;
/** @brief Text labels for the available settings categories. Indexed by ScreenStateSettings. */
extern char* settings_options[SETTINGS_OPTIONS_COUNT];
/** @brief The currently selected settings category (e.g., PWM, Current). */
extern ScreenStateSettings state_settings;
/** @brief The cursor position or selected item within a settings screen. */
extern int8_t settings_pos;
/** @brief A secondary cursor or state variable for multi-level settings (e.g., toggling warning/shutdown). */
extern int8_t settings_pos2;

/** @brief Represents the navigation level within a settings category. */
typedef enum {
	State_Settings_Menu_Channels = 0, //!< Selecting a channel to configure.
	State_Settings_Menu_Sensor,       //!< Selecting a sensor within the channel.
	State_Settings_Menu_Settings      //!< Adjusting the setting value.
} ScreenStateSettingsMenu;
/** @brief The current navigation level in the settings menu. */
extern ScreenStateSettingsMenu state_settings_menu;
/** @brief The index of the channel being configured in the settings menu. */
extern uint8_t state_settings_menu_channel;
/** @brief The index of the sensor being configured (if applicable). */
extern uint8_t state_settings_menu_sensor;

#endif /* INC_CONTROLLER_GLOBALS_H_ */
