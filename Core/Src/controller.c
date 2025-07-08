/*
 * controller.c
 *
 *  Created on: Jul 8, 2025
 *      Author: kiril
 */

#include "project_types.h"


PowerChannel* channels;
Button* additional_buttons;
FanController* fan_controllers;

void init_controller(PowerChannel* ch, Button* buttons, FanController* fans){
	channels = ch;
	additional_buttons = buttons;
	fan_controllers = fans;
}
