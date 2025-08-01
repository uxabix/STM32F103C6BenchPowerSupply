/**
 * @file ads1115.c
 * @brief Driver implementation for the ADS1115 external ADC.
 * @author kiril
 * @date Jul 7, 2025
 */

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "defines.h"


/**
 * @brief Gets the Full-Scale Range (FSR) voltage for a given PGA setting.
 * @param pga_bits The 3-bit value for the PGA setting.
 * @return The FSR voltage in Volts.
 */
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
        default:    return 4.096f; // Default safe value
    }
}

/**
 * @brief Selects the MUX bits for differential channel measurements.
 * @param pos_channel The positive input channel (0-3).
 * @param neg_channel The negative input channel (0-3).
 * @return The 16-bit value for the MUX field in the config register.
 */
static uint16_t get_mux_bits(uint8_t pos_channel, uint8_t neg_channel) {
    // Note: The ADS1115 supports specific differential pairs.
    // AIN0-AIN1 = 0, AIN0-AIN3 = 1, AIN1-AIN3 = 2, AIN2-AIN3 = 3
    if (pos_channel == 0 && neg_channel == 1) return 0b000 << 12;
    if (pos_channel == 0 && neg_channel == 3) return 0b001 << 12;
    if (pos_channel == 1 && neg_channel == 3) return 0b010 << 12;
    if (pos_channel == 2 && neg_channel == 3) return 0b011 << 12;

    // Default to AIN0-AIN1 for unsupported pairs.
    return 0b000 << 12;
}

/**
 * @brief Initializes the ADS1115 for continuous differential measurement.
 * @note This only needs to be called once at startup if CONTINUOUS_MODE is enabled.
 */
void ads1115_init_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel) {
#if CONTINUOUS_MODE
    uint16_t config = 0;
    config |= get_mux_bits(pos_channel, neg_channel); // MUX setting
    config |= (ADS1115_PGA << 9);  // PGA gain
    config |= (0 << 8);      // Continuous conversion mode
    config |= (0b100 << 5);  // Data rate: 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);
    HAL_I2C_Master_Transmit(hi2c, addr, buf, 3, HAL_MAX_DELAY);
#endif
}

/**
 * @brief Initializes the ADS1115 for continuous single-ended measurement.
 * @note This only needs to be called once at startup if CONTINUOUS_MODE is enabled.
 */
void ads1115_init_single_continuous(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel) {
#if CONTINUOUS_MODE
    if (channel > 3) return;

    uint16_t config = 0;
    config |= (0b100 | channel) << 12;  // MUX for AINx vs GND
    config |= (ADS1115_PGA << 9);  // PGA gain
    config |= (0 << 8);      // Continuous conversion mode
    config |= (0b100 << 5);  // Data rate: 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);

    HAL_I2C_Master_Transmit(hi2c, addr, buf, 3, HAL_MAX_DELAY);
#endif
}

/**
 * @brief Reads a differential value from the ADS1115.
 * @details In continuous mode, it just reads the conversion register. In single-shot
 *          mode, it initiates a conversion and waits for it to complete.
 * @return The raw 16-bit signed ADC value, or -9999 on I2C error.
 */
int16_t ads1115_read_diff(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t pos_channel, uint8_t neg_channel){
#if !CONTINUOUS_MODE
    // Single-shot measurement
    uint16_t config = 0;
    config |= 1 << 15;  // Start single conversion (OS bit)
    config |= get_mux_bits(pos_channel, neg_channel);
    config |= (ADS1115_PGA << 9);  // PGA
    config |= (1 << 8);      // Single-shot mode
    config |= (0b100 << 5);  // 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);

    if (HAL_I2C_Master_Transmit(hi2c, addr, buf, 3, HAL_MAX_DELAY) != HAL_OK)
        return -9999;

    // Wait for conversion to complete by polling the OS bit
    HAL_Delay(8); // Minimum delay for 128SPS is ~7.8ms // TO REPLACE
    uint8_t cfg_reg = ADS1115_REG_CONFIG;
    uint8_t cfg[2];
    uint16_t status = 0;
    do {
        HAL_I2C_Master_Transmit(hi2c, addr, &cfg_reg, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(hi2c, addr, cfg, 2, HAL_MAX_DELAY);
        uint16_t status = (cfg[0] << 8) | cfg[1];
    } while ((status & 0x8000) == 0); // Wait until OS bit is 1
#endif

    // Read the result from the conversion register
    uint8_t reg = ADS1115_REG_CONV;
    uint8_t data[2];
    if (HAL_I2C_Master_Transmit(hi2c, addr, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return -9999;
    if (HAL_I2C_Master_Receive(hi2c, addr, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return -9999;

    int16_t raw = (data[0] << 8) | data[1];

    return raw;
}

/**
 * @brief Reads a single-ended value from the ADS1115.
 * @details In continuous mode, it just reads the conversion register. In single-shot
 *          mode, it initiates a conversion and waits for it to complete.
 * @return The raw 16-bit signed ADC value, or -9999 on I2C error.
 */
int16_t ads1115_read_single(uint8_t addr, I2C_HandleTypeDef* hi2c, uint8_t channel){
	if (channel > 3) return -9999;

#if !CONTINUOUS_MODE
    // Single-shot measurement
	uint16_t config = 0;
	config |= (1 << 15);  // Start single-shot
	config |= (0b100 | channel) << 12;  // MUX for AINx vs GND
	config |= (ADS1115_PGA << 9);  // PGA
	config |= (1 << 8);      // Single-shot mode
	config |= (0b100 << 5);  // 128 SPS
	config |= 3;             // Disable comparator

	uint8_t buf[3];
	buf[0] = ADS1115_REG_CONFIG;
	buf[1] = (uint8_t)(config >> 8);
	buf[2] = (uint8_t)(config & 0xFF);

	if (HAL_I2C_Master_Transmit(hi2c, addr, buf, 3, HAL_MAX_DELAY) != HAL_OK)
		return -9999;

    // Wait for conversion to complete by polling the OS bit
    HAL_Delay(8); // Minimum delay for 128SPS is ~7.8ms // TO REPLACE
	uint8_t cfg_reg = ADS1115_REG_CONFIG;
	uint8_t cfg[2];
	uint16_t status = 0;
    do {
        HAL_I2C_Master_Transmit(hi2c, addr, &cfg_reg, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(hi2c, addr, cfg, 2, HAL_MAX_DELAY);
		uint16_t status = (cfg[0] << 8) | cfg[1];
    } while ((status & 0x8000) == 0); // Wait until OS bit is 1
#endif

	// Read the result from the conversion register
	uint8_t reg = ADS1115_REG_CONV;
	uint8_t data[2];

	if (HAL_I2C_Master_Transmit(hi2c, addr, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
		return -9999;
	if (HAL_I2C_Master_Receive(hi2c, addr, data, 2, HAL_MAX_DELAY) != HAL_OK)
		return -9999;

	int16_t raw = (data[0] << 8) | data[1];

	return raw;
}
