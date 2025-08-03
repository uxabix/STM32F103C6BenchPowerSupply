/**
 * @file buttons.c
 * @brief Implements button state detection logic.
 * @author kiril
 * @date Jul 8, 2025
 */

#include "buttons.h"
#include "defines.h"
#include "project_types.h"

/**
 * @brief Prints the current button event for debugging purposes.
 * @details This function is useful for diagnostics, sending the button's
 *          current state to a serial console. The delays are to prevent
 *          flooding the output and can be removed.
 * @param btn Pointer to the Button structure to inspect.
 */
#if DEBUG
void print_event(Button* btn){
	switch (btn->state){
	case BUTTON_IDLE:
		// debug_printf("STATE: IDLE\r\n"); // Too verbose for normal operation
		break;
	case BUTTON_SHORT_PRESS:
		debug_printf("EVENT: SHORT PRESS\r\n");
		HAL_Delay(100);
		break;
	case BUTTON_LONG_PRESS:
		debug_printf("EVENT: LONG PRESS\r\n");
		HAL_Delay(100);
		break;
	case BUTTON_RELEASED:
		debug_printf("EVENT: RELEASED\r\n");
		HAL_Delay(100); // Optional: can be removed
		break;
    default:
        // No output for other states like DEBOUNCE
        break;
	}
}
#endif

/**
 * @brief Updates the state machine for a single button.
 * @details This function implements debouncing and detects short and long press events.
 *          It should be called periodically for each button (e.g., in a SysTick handler
 *          or a main loop). When a press event is detected (short or long), the `event`
 *          field of the Button struct is set. The main application logic should check
 *          this field and reset it to BUTTON_IDLE after handling it.
 * @param btn Pointer to the Button structure to update.
 * @param current_time The current system time in milliseconds (e.g., from HAL_GetTick()).
 */
void button_update(Button* btn, uint32_t current_time) {
    // Read pin state, inverting logic for normally closed buttons.
    bool pin_state = btn->normally_open ?
    		(HAL_GPIO_ReadPin(btn->pin.port, btn->pin.pin) == GPIO_PIN_RESET) :
    		(HAL_GPIO_ReadPin(btn->pin.port, btn->pin.pin) == GPIO_PIN_SET);

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
                    // Debounce successful, button is held down.
                    btn->state = BUTTON_SHORT_PRESS;
                    btn->last_change_time = current_time;
                } else {
                    // It was just noise, go back to idle.
                    btn->state = BUTTON_IDLE;
                }
            }
            break;

        case BUTTON_SHORT_PRESS:
            if (!pin_state) {
                // Button released before long press duration.
            	btn->event = BUTTON_SHORT_PRESS;
                btn->state = BUTTON_RELEASED;
            } else if ((current_time - btn->last_change_time) >= btn->long_press_ms) {
                // Button has been held long enough for a long press.
                btn->state = BUTTON_LONG_PRESS;
                btn->event = BUTTON_LONG_PRESS;
            }
            break;

        case BUTTON_LONG_PRESS:
            if (!pin_state) {
                // Button released after a long press.
                btn->state = BUTTON_RELEASED;
            }
            break;

        case BUTTON_RELEASED:
            // A one-shot state to signal release. The main logic will consume the event
            // and we return to idle to detect the next press.
            btn->state = BUTTON_IDLE;
            break;
    }
}

/**
 * @brief Updates all buttons in a given array.
 * @details This is a convenience function that iterates through an array of buttons
 *          and calls button_update() for each one, passing the current system time.
 * @param buttons An array of pointers to Button structures.
 * @param count The number of buttons in the array.
 */
void update_buttons(Button** buttons, uint8_t count){
    uint32_t now = HAL_GetTick();
	for (uint8_t i = 0; i < count; i++){
		button_update(buttons[i], now);
	}
}
