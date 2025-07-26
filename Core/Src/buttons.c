/*
 * buttons.c
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#include "defines.h"
#include "project_types.h"

void print_event(Button* btn){
	switch (btn->state){
	case BUTTON_IDLE:
		printf("STATE: IDLE\r\n");
		break;
	case BUTTON_SHORT_PRESS:
		printf("STATE: SHORT PRESS\r\n");
		HAL_Delay(100);
		break;
	case BUTTON_LONG_PRESS:
		printf("STATE: LONG PRESS\r\n");
		HAL_Delay(100);
		break;
	case BUTTON_RELEASED:
		printf("RELEASED\r\n");
		HAL_Delay(100);
	}
}

void button_update(Button* btn, uint32_t current_time) {
    bool pin_state = btn->normally_open ?
    		HAL_GPIO_ReadPin(btn->pin.port, btn->pin.pin) == GPIO_PIN_RESET :
    		HAL_GPIO_ReadPin(btn->pin.port, btn->pin.pin) == GPIO_PIN_SET;
    switch (btn->state) {
        case BUTTON_IDLE:
            if (pin_state) {
                btn->state = BUTTON_DEBOUNCE;
                btn->last_change_time = current_time;
            }
            break;

        case BUTTON_DEBOUNCE:
            if ((current_time - btn->last_change_time) >= btn->debounce_ms) {
                if (pin_state) {
                    btn->state = BUTTON_SHORT_PRESS;
                    btn->last_change_time = current_time;
                } else {
                    btn->state = BUTTON_IDLE;
                }
            }
            break;

        case BUTTON_SHORT_PRESS:
            if (!pin_state) {
            	btn->event = btn->state;
                btn->state = BUTTON_RELEASED;
            } else if ((current_time - btn->last_change_time) >= btn->long_press_ms) {
                btn->state = BUTTON_LONG_PRESS;
            }
            break;

        case BUTTON_LONG_PRESS:
            if (!pin_state) {
                btn->event = btn->state;
                btn->state = BUTTON_RELEASED;
            }
            break;

        case BUTTON_RELEASED:
            // Обработай в основном коде
            btn->state = BUTTON_IDLE;
            break;
    }
}

void update_buttons(Button** buttons, uint8_t count){
    uint32_t now = HAL_GetTick();
	for (int i = 0; i < count; i++){
		button_update(buttons[i], now);
	}
}
