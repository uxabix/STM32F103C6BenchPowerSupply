/*
 * adc_manager.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_ADC_MANAGER_H_
#define INC_ADC_MANAGER_H_

#include "project_types.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

float read_adc(const ADCInput* adc);

#endif /* INC_ADC_MANAGER_H_ */
