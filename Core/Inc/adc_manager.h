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


/**
 * @brief Initialize the ADC manager with internal and external ADC interfaces.
 *
 * This function sets up pointers to internal ADC and I2C peripherals and
 * optionally initializes an external ADS1115 ADC in continuous mode,
 * depending on compile-time configuration flags.
 *
 * @param hadc Pointer to the internal ADC handler (HAL ADC).
 * @param hi2c Pointer to the I2C handler used for external ADC communication.
 */
void init_adc_manager(ADC_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c);

/**
 * @brief Reads a voltage value from the specified ADC input.
 * @param adc Pointer to the ADCInput structure describing the ADC to read from.
 * @return The calculated voltage in Volts.
 */
float get_voltage(ADCInput* adc);

#endif /* INC_ADC_MANAGER_H_ */
