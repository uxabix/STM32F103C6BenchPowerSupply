/**
 * @file adc_manager.c
 * @brief Implements reading from both internal and external ADCs.
 * @author kiril
 * @date Jul 7, 2025
 */

#include "adc_manager.h"
#include "ads1115.h"


/**
 * @brief Pointer to the internal ADC (HAL ADC).
 *
 * This pointer is assigned during initialization and used
 * throughout the ADC manager for accessing internal ADC functions.
 */
static ADC_HandleTypeDef *m_hadc;
/**
 * @brief Pointer to the I2C interface for external ADC communication.
 *
 * This pointer is set during initialization and is required
 * to communicate with the ADS1115 external ADC over I2C.
 */
static I2C_HandleTypeDef *m_hi2c;


/**
 * @brief Initialize the ADC manager and optionally configure external ADC.
 *
 * Assigns internal references to the provided ADC and I2C handles.
 * If `CONTINUOUS_MODE` is defined, configures the external ADS1115 ADC
 * in continuous mode. The mode (differential or single-ended) is determined
 * by the `DIFFERENTIAL_MODE` macro.
 *
 * - If CONTINUOUS_MODE && DIFFERENTIAL_MODE:
 *     - Initializes ADS1115 in continuous differential mode on AIN0-AIN1.
 * - If CONTINUOUS_MODE && !DIFFERENTIAL_MODE:
 *     - Initializes ADS1115 in continuous single-ended mode (example: channel 1).
 *
 * @param hadc Pointer to internal ADC handle (HAL ADC).
 * @param hi2c Pointer to I2C handle used to communicate with ADS1115.
 */
void init_adc_manager(ADC_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c){
	m_hadc = hadc;
	m_hi2c = hi2c;
    // Initialize external ADC for continuous mode if configured
#if CONTINUOUS_MODE && DIFFERENTIAL_MODE
	ads1115_init_continuous(ADS1115_ADDR, m_hi2c, 0, 1);  // AIN0-AIN1
#endif
#if CONTINUOUS_MODE && !DIFFERENTIAL_MODE
	ads1115_init_single_continuous(1); // Example: single-ended channel 1
#endif
}

/**
 * @brief Reads a raw ADC value from an internal STM32 ADC.
 * @param adc Pointer to the ADCInput structure.
 * @return Raw 12-bit value (0–4095) from the selected channel. Returns 0 on error.
 */
static int16_t read_internal_adc(ADCInput* adc){
	if (m_hadc == NULL) return 0; // Safety check: return 0 if m_hadc has not been initialized
	// if (m_hadc2 == NULL) return 0; // Add if ADC2 is used

	// Handle internal STM32 ADC
	ADC_HandleTypeDef* adc_handler = NULL;
	switch (adc->adc_id) {
		case 0:
			adc_handler = m_hadc;
			break;
		// case 1: hadc = &hadc2; break; // Add if ADC2 is used
		default:
			// This case should not be reached with proper configuration
			debug_printf("ADC not initialized!\r\n");
			return 0;
	}

	// Configure the specific channel for a single conversion
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = adc->channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5; // A moderate sampling time to balance speed and accuracy.

	if (HAL_ADC_ConfigChannel(adc_handler, &sConfig) != HAL_OK) {
		return 0;
	}
	HAL_ADC_Start(adc_handler);
	HAL_ADC_PollForConversion(m_hadc, 10); // 10ms timeout
	int16_t raw = HAL_ADC_GetValue(adc_handler);
	HAL_ADC_Stop(adc_handler);
	adc->value = raw;
	return raw;
}

/**
 * @brief Reads a raw ADC value from an external ADC (e.g. ADS1115).
 * @param adc Pointer to the ADCInput structure.
 * @return Raw 16-bit value from the ADC. Returns 0 on failure or unsupported ID.
 */
static int16_t read_external_adc(ADCInput* adc){
	if (m_hi2c == NULL) return 0; // Safety check: return 0 if m_hi2c has not been initialized
	// Handle external ADC (assuming ADS1115)
	if (adc->adc_id == 1) { // adc_id 1 is hardcoded for the ADS1115
		int16_t raw;
		if (adc->mode == ADC_DIFFERENTIAL) {
			raw = ads1115_read_diff(ADS1115_ADDR, m_hi2c, adc->pos_channel, adc->neg_channel);
		} else {
			raw = ads1115_read_single(ADS1115_ADDR, m_hi2c, adc->channel);
		}
		adc->value = raw;
		return raw;
	}

	return 0;
}

/**
 * @brief Reads a raw value from the specified ADC input.
 * @details This function routes the read request to the correct ADC source
 *          (internal or external) and updates the cached value in the input.
 * @param adc Pointer to the ADCInput structure describing the ADC to read from.
 * @return The raw value from the ADC. Returns 0 on error or unknown source.
 */
static int16_t read_adc(ADCInput* adc) {
	if (adc == NULL) return 0;

	if (adc->source == ADC_INTERNAL) {
        return read_internal_adc(adc);
    }
	if (adc->source == ADC_EXTERNAL) {
        return read_external_adc(adc);
    }

    return 0; // Return 0 if source is not recognized
}

/**
 * @brief Reads a voltage value from the specified ADC input.
 * @details This function first calls `read_adc` to get a raw value, then converts
 *          it to a voltage based on the ADC's properties (reference voltage,
 *          resolution, and gain).
 * @param adc Pointer to the ADCInput structure describing the ADC to read from.
 * @return The calculated voltage in Volts.
 */
float get_voltage(ADCInput* adc){
	read_adc(adc);
	float voltage = 0.0f;
	if (adc->source == ADC_INTERNAL){
        const float vref = 3.3f;      // System reference voltage (VDDA)
		const float adc_max = 4095.0f;  // Max value for a 12-bit ADC (2^12 - 1)
		voltage = (adc->value / adc_max) * vref;
	} else if (adc->source == ADC_EXTERNAL){
        // For ADS1115, the FSR depends on the PGA setting
		const float adc_max = 32767.0f; // Max value for a 16-bit signed ADC (2^15 - 1)
	    voltage = adc->value * (ads1115_get_fsr(ADS1115_PGA) / adc_max);
	}

	return voltage;
}
