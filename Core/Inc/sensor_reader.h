/**
 * @file sensor_reader.h
 * @brief Functions to read and process data from various sensors.
 * @author kiril
 * @date Jul 7, 2025
 */

#ifndef INC_SENSOR_READER_H_
#define INC_SENSOR_READER_H_

#include "project_types.h"

/**
 * @brief Updates temperature readings for all specified channels.
 * @details Reads ADC values, converts them to temperature, and updates the
 *          warning/shutdown status for each temperature sensor in the channels.
 * @param channels An array of pointers to PowerChannel structures.
 * @param count The number of channels in the array.
 */
void update_temperatures(PowerChannel** channels, uint8_t count);

/**
 * @brief Updates current readings for all specified channels.
 * @details Reads ADC values, converts them to current, and updates the
 *          warning/shutdown status for each current sensor in the channels.
 * @param channels An array of pointers to PowerChannel structures.
 * @param count The number of channels in the array.
 */
void update_currents(PowerChannel** channels, uint8_t count);

/**
 * @brief Updates voltage readings for all specified channels.
 * @details Reads ADC values, converts them to voltage, and updates the
 *          warning/shutdown status for each voltage sensor in the channels.
 * @param channels An array of pointers to PowerChannel structures.
 * @param count The number of channels in the array.
 */
void update_voltages(PowerChannel** channels, uint8_t count);


#endif /* INC_SENSOR_READER_H_ */
