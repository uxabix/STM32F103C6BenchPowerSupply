/*
 * controller_buttons.c
 *
 *  Created on: Aug 3, 2025
 *      Author: kiril
 */
#include <stdbool.h>
#include <inttypes.h>

#include "controller_buttons.h"
#include "controller_globals.h"
#include "project_types.h"
#include "power_channel.h"


static void normal_behaviour(Button* button, uint8_t index, bool is_channel_button){
	switch (button->event) {
		case BUTTON_SHORT_PRESS:
			refresh_screen = true;
			if (is_channel_button) {
				toggle_channel(button_channels[index]);
			} else if (SETTINGS_BUTTON) {
				state = State_Main;
			}
			break;
		case BUTTON_LONG_PRESS:
			refresh_screen = true;
			if (is_channel_button && (state == State_Main || state == State_Channel)) {
				if (displayed_channel == index && state == State_Channel) {
					state = State_Main;
				} else {
					state = State_Channel;
					displayed_channel = index;
				}
			} else if (SETTINGS_BUTTON) {
				state = State_Settings;
				state_settings = State_Settings_Main;
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
static void pwm_carousel_adjust_digit(uint8_t channel_idx, uint8_t active_digit, bool increase) {
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

static void settings_behaviour(Button* button, uint8_t index, bool is_channel_button){
	switch (button->event) {
		case BUTTON_SHORT_PRESS:
			refresh_screen = true;
			if (state_settings != State_Settings_Main && state_settings_menu == State_Settings_Menu_Channels){
				if (state_settings == State_Settings_PWM) state_settings_menu = State_Settings_Menu_Settings;
				else state_settings_menu = State_Settings_Menu_Sensor;
				state_settings_menu_channel = settings_pos;
			} else if (SETTINGS_BUTTON_Decrease && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings){
				pwm_carousel_adjust_digit(state_settings_menu_channel, 2 - settings_pos, false);
			} else if (SETTINGS_BUTTON_Increase && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings){
				pwm_carousel_adjust_digit(state_settings_menu_channel, 2 - settings_pos, true);
			} else if (SETTINGS_BUTTON && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings){
				settings_pos++;
			} else if (SETTINGS_BUTTON_Decrease) {
				settings_pos--;
			} else if (SETTINGS_BUTTON_Increase) {
				settings_pos++;
			} else if (state_settings == State_Settings_Main && SETTINGS_BUTTON){
				state_settings = settings_pos;
			}
			break;
		case BUTTON_LONG_PRESS:
			refresh_screen = true;
			if (SETTINGS_BUTTON) {
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
void buttons_action() {
	for (uint8_t i = 0; i < button_channels_count; i++) {
		handle_button_event(button_channels[i]->button, i, true);
	}
	for (uint8_t i = 0; i < additional_buttons_count; i++) {
		handle_button_event(additional_buttons[i], i, false);
	}
}
