/**
 * @file adc_manager.c
 * @brief Implements reading from both internal and external ADCs.
 * @author kiril
 * @date Jul 7, 2025
 */

#include "adc_manager.h"
#include "ads1115.h"

/**
 * @brief Reads a raw value from the specified ADC input.
 * @details This function handles routing the read request to either the internal
 *          STM32 ADC or an external I2C ADC like the ADS1115 based on the
 *          `ADCInput` configuration. It also updates the `value` field in the
 *          `ADCInput` struct.
 * @param adc Pointer to the ADCInput structure describing the ADC to read from.
 * @return The raw 16-bit value from the ADC. Returns 0 on failure.
 */
int16_t read_adc(ADCInput* adc) {
    if (adc->source == ADC_INTERNAL) {
        // Handle internal STM32 ADC
        ADC_HandleTypeDef* hadc = NULL;
        switch (adc->adc_id) {
            case 0:
                hadc = &hadc1;
                break;
            // case 1: hadc = &hadc2; break; // Add if ADC2 is used
            default:
                // This case should not be reached with proper configuration
                printf("ADC not initialized!\r\n");
                return 0;
        }

        // Configure the specific channel for a single conversion
        ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.Channel = adc->channel;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5; // This may need tuning

		if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
            return 0;
        }
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10); // 10ms timeout
        int16_t raw = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        adc->value = raw;
        return raw;

    } else if (adc->source == ADC_EXTERNAL) {
        // Handle external ADC (assuming ADS1115)
        if (adc->adc_id == 1) {
            int16_t raw;
            if (adc->mode == ADC_DIFFERENTIAL) {
            	raw = ads1115_read_diff(ADS1115_ADDR, &hi2c1, adc->pos_channel, adc->neg_channel);
            } else {
            	raw = ads1115_read_single(ADS1115_ADDR, &hi2c1, adc->channel);
            }
            adc->value = raw;
            return raw;
        }
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
        const float vref = 3.3f;      // System reference voltage
		const float adc_max = 4095.0f;  // 12-bit ADC
		voltage = (adc->value / adc_max) * vref;
	} else if (adc->source == ADC_EXTERNAL){
        // For ADS1115, the FSR depends on the PGA setting
		const float adc_max = 32767.0f; // 15-bit effective resolution for signed value
	    voltage = adc->value * (ads1115_get_fsr(ADS1115_PGA) / adc_max);
	}

	return voltage;
}
