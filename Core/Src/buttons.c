/*
 * buttons.c
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#include "project_types.h"

void button_update(Button* btn, uint32_t current_time) {
    uint8_t pin_state = HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET;

    switch (btn->state) {
        case BUTTON_IDLE:
            if (pin_state) {
                btn->state = BUTTON_DEBOUNCE;
                btn->last_change_time = current_time;
            }
            break;

        case BUTTON_DEBOUNCE:
            if ((current_time - btn->last_change_time) >= DEBOUNCE_TIME) {
                if (pin_state) {
                    btn->state = BUTTON_PRESSED;
                    btn->last_change_time = current_time;
                } else {
                    btn->state = BUTTON_IDLE;
                }
            }
            break;

        case BUTTON_PRESSED:
            if (!pin_state) {
                btn->state = BUTTON_RELEASED;
                btn->pressed = 1;
            } else if ((current_time - btn->last_change_time) >= LONG_PRESS_TIME) {
                btn->state = BUTTON_LONG_PRESS;
                btn->pressed = 2;
            }
            break;

        case BUTTON_LONG_PRESS:
            if (!pin_state) {
                btn->state = BUTTON_RELEASED;
            }
            break;

        case BUTTON_RELEASED:
            // Обработай в основном коде
            btn->state = BUTTON_IDLE;
            break;
    }
}

