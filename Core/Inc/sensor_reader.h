/*
 * sensor_reader.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_SENSOR_READER_H_
#define INC_SENSOR_READER_H_

#include "ads1115.h"
#include "project_types.h"

void update_temperatures(PowerChannel** channels, uint8_t count);
void update_currents(PowerChannel** channels, uint8_t count);
void update_voltages(PowerChannel** channels, uint8_t count);


#endif /* INC_SENSOR_READER_H_ */
