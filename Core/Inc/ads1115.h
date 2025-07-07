/*
 * ads1115.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

#include "stm32f1xx_hal.h"

void ads1115_init_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel);
void ads1115_init_single_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel);
float ads1115_read_diff(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel);
float ads1115_read_single(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel);

#endif /* INC_ADS1115_H_ */
