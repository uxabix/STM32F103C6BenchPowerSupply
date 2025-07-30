/**
 * @file adc_manager.h
 * @brief Manages reading from both internal and external ADCs.
 * @author kiril
 * @date Jul 7, 2025
 */

#ifndef INC_ADC_MANAGER_H_
#define INC_ADC_MANAGER_H_

#include <stdio.h>
#include "project_types.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Reads a raw value from the specified ADC input.
 * @param adc Pointer to the ADCInput structure describing the ADC to read from.
 * @return The raw 16-bit value from the ADC.
 */
int16_t read_adc(ADCInput* adc);

/**
 * @brief Reads a voltage value from the specified ADC input.
 * @param adc Pointer to the ADCInput structure describing the ADC to read from.
 * @return The calculated voltage in Volts.
 */
float get_voltage(ADCInput* adc);

#endif /* INC_ADC_MANAGER_H_ */
