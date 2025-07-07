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

extern PowerChannel channels[MAX_CHANNELS];

PowerChannel* update_all_temperatures(void);
void update_all_currents_and_voltages(void);


#endif /* INC_SENSOR_READER_H_ */
