/**
 * @file ads1115.h
 * @brief Driver for the ADS1115 external ADC.
 * @author kiril
 * @date Jul 7, 2025
 *
 * @details This driver provides functions to initialize and read from the
 * ADS1115 16-bit I2C ADC. It supports both single-ended and differential
 * measurements, as well as single-shot and continuous modes.
 */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

#include "stm32f1xx_hal.h"

/**
 * @brief Gets the Full-Scale Range (FSR) voltage for a given PGA setting.
 * @param pga_bits The 3-bit value for the Programmable Gain Amplifier setting from the config register.
 * @return The FSR voltage in Volts.
 */
float ads1115_get_fsr(uint16_t pga_bits);

/**
 * @brief Initializes the ADS1115 for continuous differential measurement mode.
 * @param addr The I2C address of the ADS1115.
 * @param hi2c Pointer to the I2C handle.
 * @param pos_channel The positive input channel (e.g., 0 for AIN0).
 * @param neg_channel The negative input channel (e.g., 1 for AIN1).
 */
void ads1115_init_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel);

/**
 * @brief Initializes the ADS1115 for continuous single-ended measurement mode.
 * @param addr The I2C address of the ADS1115.
 * @param hi2c Pointer to the I2C handle.
 * @param channel The input channel to measure against GND (0-3 for AIN0-AIN3).
 */
void ads1115_init_single_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel);

/**
 * @brief Initializes the ADS1115 for continuous single-ended measurement mode.
 * @param addr The I2C address of the ADS1115.
 * @param hi2c Pointer to the I2C handle.
 * @param channel The input channel to measure against GND (0-3 for AIN0-AIN3).
 */
int16_t ads1115_read_diff(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel);

/**
 * @brief Reads a single-ended value from the ADS1115.
 * @param addr The I2C address of the ADS1115.
 * @param hi2c Pointer to the I2C handle.
 * @param channel The input channel to measure against GND (0-3).
 * @return The raw 16-bit signed ADC value.
 */
int16_t ads1115_read_single(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel);

#endif /* INC_ADS1115_H_ */
