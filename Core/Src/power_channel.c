/*
 * power_channel.c
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#include "power_channel.h"

PowerChannel* get_channel_with_max_current() {
    PowerChannel* max_channel = NULL;
    float max_current = -1.0f;

    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        PowerChannel* ch = &channels[i];
        if (ch->current_sensor == NULL) continue;
        float current = ch->current_sensor->last_value;
        if (ch->current_sensor != NULL && current > max_current) {
        	max_current = current;
			max_channel = ch;
        }
    }

    printf("Max current x100: %d", (int)(max_current * 100));
    return max_channel;  // NULL, если ни у одного канала нет датчика тока
}

PowerChannel* get_channel_with_max_voltage() {
    PowerChannel* max_channel = NULL;
    float max_voltage = -1.0f;

    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        PowerChannel* ch = &channels[i];
        if (ch->voltage_sensor == NULL) continue;
        float voltage = ch->voltage_sensor->last_value;
        if (voltage > max_voltage) {
        	max_voltage = voltage;
			max_channel = ch;
        }
    }
    printf("Max voltage x100: %d", (int)(max_voltage * 100));
    return max_channel;  // NULL, если ни один канал не измеряет напряжение
}
