/**
 * @file sensor_reader.c
 * @brief Implements functions to read and process sensor data.
 * @author kiril
 * @date Jul 7, 2025
 */

#include <math.h>

#include "sensor_reader.h"
#include "adc_manager.h"
#include "power_channel.h"

void init_sensors(ADC_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c){
    // Initialize internal and external ADCs
	init_adc_manager(hadc, hi2c);

    // Future sensor initializations can be added here
}

/**
 * @brief Calculates temperature in Celsius from an NTC thermistor's resistance.
 * @details This function uses the Steinhart-Hart equation (simplified Beta model)
 *          to convert the measured resistance of a thermistor into a temperature reading.
 * @param r_therm The measured resistance of the thermistor in Ohms.
 * @param nominal_resistance The thermistor's nominal resistance (e.g., 10000 Ohms).
 * @param nominal_temperature The nominal temperature in Kelvin (e.g., 298.15K for 25В°C).
 * @param beta The thermistor's Beta coefficient (e.g., 3435).
 * @return The calculated temperature in degrees Celsius. Returns absolute zero on error.
 */
static float thermistor_to_celsius(float r_therm, float nominal_resistance,
                                   float nominal_temperature, float beta) {
    if (r_therm <= 0.0f) {
        return -273.15f; // Invalid resistance, return absolute zero.
    }

    // Steinhart-Hart equation (Beta model)
    float steinhart;
    steinhart = r_therm / nominal_resistance;     // (R/R0)
    steinhart = logf(steinhart);                  // ln(R/R0)
    steinhart /= beta;                            // 1/B * ln(R/R0)
    steinhart += 1.0f / nominal_temperature;      // 1/T0 + 1/B * ln(R/R0)
    steinhart = 1.0f / steinhart;                 // Inverse to get T in Kelvin
    steinhart -= 273.15f;                         // Convert Kelvin to Celsius

    return steinhart;
}

/**
 * @brief Updates temperature readings for all specified channels.
 */
void update_temperatures(PowerChannel** channels, uint8_t count) {
    for (int ch = 0; ch < count; ++ch) {
        PowerChannel* channel = channels[ch];
        bool channel_shutdown = false;
        bool channel_warning = false;

        for (int i = 0; i < channel->temp_sensor_count; ++i) {
            TemperatureSensor* sensor = &channel->temp_sensors[i];
            float voltage = get_voltage(&sensor->adc);
            const float vref = 3.3f;

            // Calculate thermistor resistance from voltage divider formula:
            // Vout = Vref * R_therm / (R_therm + R_series)
            // R_therm = R_series * Vout / (Vref - Vout)
			float r_therm = sensor->series_resistor * voltage / (vref - voltage);

			float temp_c = thermistor_to_celsius(
				    r_therm,
				    sensor->nominal_resistance,
				    sensor->nominal_temperature,
				    sensor->beta
				);

            sensor->last_value = (int8_t)roundf(temp_c);
            sensor->warning_triggered = (sensor->last_value >= sensor->warning_threshold);
            sensor->shutdown_triggered = (sensor->last_value >= sensor->shutdown_threshold);

            // Aggregate warning/shutdown status for the entire channel
            if (sensor->shutdown_triggered) {
                channel_shutdown = true;
            }
            if (sensor->warning_triggered || sensor->last_value < TEMPERATURE_ERROR) {
                channel_warning = true;
            }
        }

        // Set channel state based on aggregated sensor status. Use |= to avoid clearing
        // a flag set by another sensor type (e.g., current sensor).
        channel->in_shutdown_state |= channel_shutdown;
        channel->in_warning_state |= channel_warning;

        if (channel->in_shutdown_state) {
            disactivate_channel(channel);
        }
    }
}

/**
 * @brief Updates current readings for all specified channels.
 */
void update_currents(PowerChannel** channels, uint8_t count) {
    for (uint8_t ch = 0; ch < count; ++ch) {
        PowerChannel* channel = channels[ch];
        if (channel->current_sensor != NULL) {
            CurrentSensor* cs = channel->current_sensor;
            // Voltage across shunt is read, then current is I = V/R
            float current = get_voltage(&cs->adc) / cs->adc.conversion_factor;
            cs->last_value = current;

            cs->warning_triggered = (current > cs->warning_threshold);
            cs->shutdown_triggered = (current > cs->shutdown_threshold);

            channel->in_shutdown_state |= cs->shutdown_triggered;
            channel->in_warning_state |= cs->warning_triggered;

            if (channel->in_shutdown_state) {
                disactivate_channel(channel);
            }
        }
    }
}

/**
 * @brief Updates voltage readings for all specified channels.
 */
void update_voltages(PowerChannel** channels, uint8_t count){
    for (uint8_t ch = 0; ch < count; ++ch) {
		PowerChannel* channel = channels[ch];
		if (channel->voltage_sensor != NULL) {
			VoltageSensor* vs = channel->voltage_sensor;
            // Voltage is read from a divider, so we multiply by the ratio
			float voltage = get_voltage(&vs->adc) * vs->divider_ratio;

			vs->last_value = voltage;
			vs->overvoltage_triggered = (voltage > vs->overvoltage_threshold);
			vs->undervoltage_triggered = (voltage < vs->undervoltage_threshold);

			channel->in_shutdown_state |= vs->overvoltage_triggered;
			channel->in_warning_state |= vs->undervoltage_triggered;

			if (channel->in_shutdown_state){
				disactivate_channel(channel);
			}
		}
    }
}
