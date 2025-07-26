/*
 * sensor_reader.c
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#include "adc_manager.h"
#include "sensor_reader.h"
#include "power_channel.h"

/**
 * Вычисляет температуру в градусах Цельсия по сопротивлению термистора.
 *
 * @param r_therm Сопротивление термистора в Омах
 * @param nominal_resistance Номинальное сопротивление (обычно 10k) при 25°C
 * @param nominal_temperature Номинальная температура в Кельвинах (обычно 298.15)
 * @param beta Параметр β термистора (например, 3435)
 * @return Температура в °C
 */
float thermistor_to_celsius(float r_therm, float nominal_resistance,
                            float nominal_temperature, float beta) {
    if (r_therm <= 0.0f) return -273.15f;  // ошибка: невозможное сопротивление

    float inv_T = (1.0f / nominal_temperature) +
                  (1.0f / beta) * logf(r_therm / nominal_resistance);

    float temp_K = 1.0f / inv_T;
    return temp_K - 273.15f;
}

void update_temperatures(PowerChannel** channels, uint8_t count) {
	PowerChannel* maxTempChannel = NULL;
	float maxTempCoefficient = 0;
    for (int ch = 0; ch < count; ++ch) {
        PowerChannel* channel = channels[ch];
        for (int i = 0; i < channel->temp_sensor_count; ++i) {
            TemperatureSensor* sensor = &channel->temp_sensors[i];
            float voltage = get_voltage(&sensor->adc);
            float vref = 3.3f;
			// Делитель напряжения: Vs = Vcc * R_therm / (R_therm + R_fixed)
			float r_therm = sensor->series_resistor * voltage / (vref - voltage);

			float temp_c = thermistor_to_celsius(
				    r_therm,
				    sensor->nominal_resistance,
				    sensor->nominal_temperature,
				    sensor->beta
				);

            sensor->last_value = temp_c;
            sensor->warning_triggered = (temp_c >= sensor->warning_threshold);
            sensor->shutdown_triggered = (temp_c >= sensor->shutdown_threshold);
			if (temp_c / sensor->shutdown_threshold > maxTempCoefficient){
				maxTempChannel = channels[ch];
				maxTempCoefficient = temp_c / sensor->shutdown_threshold;
			}
            if (sensor->shutdown_triggered) {
                channel->in_shutdown_state = 1;
                disactivate_channel(channel);
            } else{
                channel->in_shutdown_state = 0;
            }
            if (sensor->warning_triggered || temp_c < TEMPERATURE_ERROR) {
                channel->in_warning_state = 1;
            } else {
                channel->in_warning_state = 0;
            }
        }
    }
}

void update_currents(PowerChannel** channels, uint8_t count) {
    for (uint8_t ch = 0; ch < count; ++ch) {
        PowerChannel* channel = channels[ch];
        if (channel->current_sensor != NULL) {
            CurrentSensor* cs = channel->current_sensor;
            float current = get_voltage(&cs->adc) / cs->adc.conversion_factor;
            cs->last_value = current;
            cs->warning_triggered = (current > cs->warning_threshold);
            cs->shutdown_triggered = (current > cs->shutdown_threshold);
            if (cs->shutdown_triggered) {
                channel->in_shutdown_state = 1;
                disactivate_channel(channel);
            } else{
                channel->in_shutdown_state = 0;
            }
            if (cs->warning_triggered) {
                channel->in_warning_state = 1;
            } else {
                channel->in_warning_state = 0;
            }
        }
    }
}

void update_voltages(PowerChannel** channels, uint8_t count){
    for (uint8_t ch = 0; ch < count; ++ch) {
		PowerChannel* channel = channels[ch];
		if (channel->voltage_sensor != NULL) {
			VoltageSensor* vs = channel->voltage_sensor;
			float voltage = get_voltage(&vs->adc) / vs->divider_ratio;

			vs->last_value = voltage;
			vs->overvoltage_triggered = (voltage > vs->overvoltage_threshold);
			vs->undervoltage_triggered = (voltage < vs->undervoltage_threshold);
			if (vs->overvoltage_triggered){
				channel->in_shutdown_state = 1;
				disactivate_channel(channel);
			} else{
				channel->in_shutdown_state = 0;
			}
			if (vs->undervoltage_triggered){
				channel->in_warning_state = 1;
			} else {
				channel->in_warning_state = 0;
			}
		}
    }
}
