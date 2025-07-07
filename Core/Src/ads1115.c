/*
 * ads1115.c
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */
#include "stm32f1xx_hal.h"
#include <stdint.h>

#include "defines.h"
#include "ads1115.h"

float ads1115_get_fsr(uint16_t pga_bits) {
    switch (pga_bits) {
        case 0b000: return 6.144f;
        case 0b001: return 4.096f;
        case 0b010: return 2.048f;
        case 0b011: return 1.024f;
        case 0b100: return 0.512f;
        case 0b101: // fallthrough
        case 0b110: // fallthrough
        case 0b111: return 0.256f;
        default:    return 2.048f; // безопасное значение по умолчанию
    }
}

// Выбор MUX для дифференциальных каналов AIN0-AIN1 и т.д.
uint16_t get_mux_bits(uint8_t pos_channel, uint8_t neg_channel) {
    // Таблица соответствия (pos, neg) -> MUX (3 бита)
    // Дифференциальные режимы поддерживаются только для пар:
    // AIN0-AIN1 = 0, AIN0-AIN3 = 1, AIN1-AIN3 = 2, AIN2-AIN3 = 3
    // Другие комбинации могут не поддерживаться ADS1115

    if (pos_channel == 0 && neg_channel == 1) return 0b000 << 12; // 0x0000
    if (pos_channel == 0 && neg_channel == 3) return 0b001 << 12; // 0x1000
    if (pos_channel == 1 && neg_channel == 3) return 0b010 << 12; // 0x2000
    if (pos_channel == 2 && neg_channel == 3) return 0b011 << 12; // 0x3000

    // Если неподдерживаемый дифф. канал, вернуть ошибку или дефолт (AIN0-AIN1)
    return 0b000 << 12;
}

// Один раз вызывается при старте, если включён continuous mode
void ads1115_init_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel) {
#if CONTINUOUS_MODE
    uint16_t config = 0;
    config |= get_mux_bits(pos_channel, neg_channel);
    config |= (ADS1115_PGA << 9);  // PGA
    config |= (0 << 8);      // Continuous mode
    config |= (0b100 << 5);  // 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);
    HAL_I2C_Master_Transmit(hi2c, addr, buf, 3, HAL_MAX_DELAY);
#endif
}

void ads1115_init_single_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel) {
#if CONTINUOUS_MODE
    if (channel > 3) return;

    uint16_t config = 0;
    config |= (0b1000 + channel) << 12;  // MUX AINx vs GND
    config |= (ADS1115_PGA << 9);  // PGA
    config |= (0 << 8);      // Continuous mode
    config |= (0b100 << 5);  // 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);

    HAL_I2C_Master_Transmit(hi2c, addr, buf, 3, HAL_MAX_DELAY);
#endif
}

float ads1115_read_diff(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel){
#if !CONTINUOUS_MODE
	printf("Not continuous mode!");
    // Одноразовое измерение (single-shot)
    uint16_t config = 0;
    config |= 1 << 15;  // Start single conversion
    config |= get_mux_bits(pos_channel, neg_channel);
    config |= (ADS1115_PGA << 9);  // PGA
    config |= (1 << 8);      // Single-shot
    config |= (0b100 << 5);  // 128 SPS
    config |= 3;

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);

    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, buf, 3, HAL_MAX_DELAY) != HAL_OK)
        return -9999.0f;

    // Ждём завершения по OS-биту
    uint8_t cfg_reg = ADS1115_REG_CONFIG;
    uint8_t cfg[2];
    uint16_t timeout = 1000;
    while (timeout--) {
        HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, &cfg_reg, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDR, cfg, 2, HAL_MAX_DELAY);
        uint16_t status = (cfg[0] << 8) | cfg[1];
        if (status & (1 << 15)) break;  // OS = 1
    }
    if (timeout == 0) return -9999.0f;
#endif

    // Чтение результата
    uint8_t reg = ADS1115_REG_CONV;
    uint8_t data[2];
    if (HAL_I2C_Master_Transmit(hi2c, addr, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return -9999.0f;
    if (HAL_I2C_Master_Receive(hi2c, addr, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return -9999.0f;

    int16_t raw = (data[0] << 8) | data[1];
    float voltage = raw * (ads1115_get_fsr(ADS1115_PGA) / 32768.0f);
    printf("Voltage ads1115 diff: %d\r\n", voltage);
    return voltage;
}

float ads1115_read_single(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel){
	if (channel > 3) return -9999.0f;

#if !CONTINUOUS_MODE
	uint16_t config = 0;
	config |= (1 << 15);  // Start single-shot
	config |= (0b1000 + channel) << 12;  // MUX AINx vs GND
	config |= (ADS1115_PGA << 9);  // PGA
	config |= (1 << 8);      // Single-shot
	config |= (0b100 << 5);  // 128 SPS
	config |= 3;

	uint8_t buf[3];
	buf[0] = ADS1115_REG_CONFIG;
	buf[1] = (uint8_t)(config >> 8);
	buf[2] = (uint8_t)(config & 0xFF);

	if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, buf, 3, HAL_MAX_DELAY) != HAL_OK)
		return -9999.0f;

	// Ждём завершения по OS-биту
	uint8_t cfg_reg = ADS1115_REG_CONFIG;
	uint8_t cfg[2];
	uint16_t timeout = 1000;
	while (timeout--) {
		HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, &cfg_reg, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDR, cfg, 2, HAL_MAX_DELAY);
		uint16_t status = (cfg[0] << 8) | cfg[1];
		if (status & (1 << 15)) break;  // OS = 1
	}
	if (timeout == 0) return -9999.0f;
#endif

	// Считывание результата
	uint8_t reg = ADS1115_REG_CONV;
	uint8_t data[2];

	if (HAL_I2C_Master_Transmit(hi2c, addr, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
		return -9999.0f;
	if (HAL_I2C_Master_Receive(hi2c, addr, data, 2, HAL_MAX_DELAY) != HAL_OK)
		return -9999.0f;

	int16_t raw = (data[0] << 8) | data[1];
	float voltage = raw * (ads1115_get_fsr(ADS1115_PGA) / 32768.0f);

	return voltage;
}
