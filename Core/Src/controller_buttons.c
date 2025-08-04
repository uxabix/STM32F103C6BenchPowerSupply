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

/**
 * @brief Handles default button behavior in Main and Channel views.
 *
 * Toggles channel power or switches screen state based on short/long press.
 *
 * @param button Pointer to the button instance.
 * @param index Index of the button (same as power channel index for channel buttons).
 * @param is_channel_button Indicates whether this button is associated with a channel.
 */
static void normal_behaviour(Button* button, uint8_t index, bool is_channel_button) {
    if (button->event == BUTTON_IDLE)
        return;

    refresh_screen = true;

    if (button->event == BUTTON_SHORT_PRESS) {
        if (is_channel_button) {
            toggle_channel(button_channels[index]);
        } else if (SETTINGS_BUTTON) {
            state = State_Main;
        }
    } else if (button->event == BUTTON_LONG_PRESS) {
        if (is_channel_button && (state == State_Main || state == State_Channel)) {
            if (displayed_channel == index && state == State_Channel)
                state = State_Main;
            else {
                state = State_Channel;
                displayed_channel = index;
            }
        } else if (SETTINGS_BUTTON) {
            state = State_Settings;
            state_settings = State_Settings_Main;
        }
    }

    button->event = BUTTON_IDLE;
}

/**
 * @brief Adjusts PWM output duty cycle by modifying a specific digit in "carousel" style.
 *
 * Wraps around 0–100% and adjusts individual digits by cursor position.
 *
 * @param channel_id Index of the PWM channel.
 * @param cursor_pos Digit position to change (0: units, 1: tens, 2: hundreds).
 * @param increase true to increment digit, false to decrement.
 */
static void adjust_pwm(uint8_t channel_id, uint8_t cursor_pos, bool increase) {
    if (!pwm_channels[channel_id])
        return;

    OutputControl* output = &pwm_channels[channel_id]->output;
    uint32_t period = output->pwm_timer->Init.Period;

    // Convert current PWM value to percent (0.0–100.0)
    float value = output->pwm_inversed
                  ? 100.0f - 100.0f * output->pwm_last_value / period
                  : 100.0f * output->pwm_last_value / period;

    uint8_t percent = (uint8_t)(value + 0.5f); // Round to nearest integer

    // Choose step size depending on cursor position
    uint8_t step;
    switch (cursor_pos) {
        case 0: step = 1; break;
        case 1: step = 10; break;
        case 2: step = 100; break;
        default: return;
    }

    // Apply "carousel" behavior
    if (increase) {
        percent += step;
        if (percent > 100)
            percent = 0;
    } else {
        if (percent >= step)
            percent -= step;
        else
            percent = 100 - (100 % step);
    }

    // Convert percent back to PWM register value
    uint32_t new_pwm = output->pwm_inversed
                       ? (uint32_t)((100.0f - percent) * period / 100.0f)
                       : (uint32_t)(percent * period / 100.0f);

    output->pwm_last_value = new_pwm;
    activate_channel(pwm_channels[channel_id]);
}

/**
 * @brief Adjusts the shutdown or warning current threshold with digit-wise precision.
 *
 * Values wrap around between defined min and max bounds.
 *
 * @param channel_id Channel whose threshold is being changed.
 * @param cursor_pos Digit position (0–4): 0-1 integer, 2-4 decimal.
 * @param increase Whether to increase (true) or decrease (false) the digit.
 * @param is_shutdown true to modify shutdown threshold, false for warning.
 */
static void adjust_current(uint8_t channel_id, uint8_t cursor_pos, bool increase, bool is_shutdown) {
    if (!power_channels[channel_id] || !power_channels[channel_id]->current_sensor)
        return;

    CurrentSensor* sensor = power_channels[channel_id]->current_sensor;
    float value = is_shutdown ? sensor->shutdown_threshold : sensor->warning_threshold;

    float delta;
    switch (cursor_pos) {
        case 0: delta = 10.0f; break;
        case 1: delta = 1.0f; break;
        case 2: delta = 0.1f; break;
        case 3: delta = 0.01f; break;
        case 4: delta = 0.001f; break;
        default: return;
    }

    value += increase ? delta : -delta;

    if (value > MAX_CURRENT_THRESHOLD)
        value = MIN_CURRENT_THRESHOLD;
    else if (value < MIN_CURRENT_THRESHOLD)
        value = MAX_CURRENT_THRESHOLD;

    int32_t milliamps = (int32_t)(value * 1000.0f + 0.5f);
    if (is_shutdown)
        sensor->shutdown_threshold = milliamps / 1000.0f;
    else
        sensor->warning_threshold = milliamps / 1000.0f;
}

/**
 * @brief Handles PWM percentage adjustment in settings menu.
 *
 * Cursor position maps to digit from right to left.
 *
 * @param increase true if increasing, false if decreasing.
 */
static void handle_pwm_adjustment(bool increase) {
    adjust_pwm(state_settings_menu_channel, 2 - settings_pos, increase);
}

/**
 * @brief Handles adjustment of current threshold in settings.
 *
 * @param increase true if increasing, false if decreasing.
 */
static void handle_current_adjustment(bool increase) {
    if (settings_pos == 0) {
        // Toggle between warning/shutdown mode
        settings_pos2 += (increase ? 1 : -1);
    } else {
        // Modify current value at selected digit
        adjust_current(state_settings_menu_channel, settings_pos - 1, increase, settings_pos2 != 0);
    }
}

/**
 * @brief Interprets user input while in Settings menu to adjust values or navigate.
 *
 * Handles both PWM and current mode, and numeric or logical navigation.
 */
static void handle_settings_navigation(Button* button, uint8_t index, bool is_channel_button) {
    if (SETTINGS_BUTTON_Decrease && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings) {
        handle_pwm_adjustment(false);
    } else if (SETTINGS_BUTTON_Increase && state_settings == State_Settings_PWM && state_settings_menu == State_Settings_Menu_Settings) {
        handle_pwm_adjustment(true);
    } else if (SETTINGS_BUTTON_Decrease && state_settings == State_Settings_Current && state_settings_menu == State_Settings_Menu_Settings) {
        handle_current_adjustment(false);
    } else if (SETTINGS_BUTTON_Increase && state_settings == State_Settings_Current && state_settings_menu == State_Settings_Menu_Settings) {
        handle_current_adjustment(true);
    } else if (SETTINGS_BUTTON_Decrease) {
        settings_pos--;
    } else if (SETTINGS_BUTTON_Increase) {
        settings_pos++;
    } else if (SETTINGS_BUTTON && (state_settings == State_Settings_PWM || state_settings == State_Settings_Current) &&
               state_settings_menu == State_Settings_Menu_Settings) {
        settings_pos++;
    } else if (state_settings == State_Settings_Main && SETTINGS_BUTTON) {
        state_settings = settings_pos;
    }
}

/**
 * @brief Handles transition from channel selection to specific setting screen.
 *
 * @param index Channel index.
 * @param is_channel_button Whether this is a channel-related button.
 */
static void handle_channel_selection(uint8_t index, bool is_channel_button) {
    if (state_settings != State_Settings_Main && state_settings_menu == State_Settings_Menu_Channels) {
        if (state_settings == State_Settings_PWM || state_settings == State_Settings_Current) {
            state_settings_menu = State_Settings_Menu_Settings;
        } else {
            state_settings_menu = State_Settings_Menu_Sensor;
        }
        state_settings_menu_channel = settings_pos;
    }
}

/**
 * @brief Handles button actions when in Settings state.
 *
 * Short press navigates or changes settings, long press exits to main menu.
 *
 * @param button Button instance.
 * @param index Index of button.
 * @param is_channel_button Whether it's tied to a channel.
 */
static void settings_behaviour(Button* button, uint8_t index, bool is_channel_button) {
    if (button->event == BUTTON_IDLE)
        return;

    refresh_screen = true;

    switch (button->event) {
        case BUTTON_SHORT_PRESS:
            handle_channel_selection(index, is_channel_button);
            handle_settings_navigation(button, index, is_channel_button);
            break;

        case BUTTON_LONG_PRESS:
            if (SETTINGS_BUTTON) {
                if (state_settings == State_Settings_Main) {
                    state = State_Main;
                } else {
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
 * @brief Central event dispatcher for button logic based on current app state.
 *
 * Routes to settings or normal behavior handlers depending on global state.
 *
 * @param button Pointer to button that triggered the event.
 * @param index Button index.
 * @param is_channel_button True if related to channel.
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
 * @brief Iterates over all buttons and dispatches actions for those with pending events.
 */
void buttons_action() {
    for (uint8_t i = 0; i < button_channels_count; i++) {
        handle_button_event(button_channels[i]->button, i, true);
    }
    for (uint8_t i = 0; i < additional_buttons_count; i++) {
        handle_button_event(additional_buttons[i], i, false);
    }
}
