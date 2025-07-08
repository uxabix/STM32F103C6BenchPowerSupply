/*
 * adc_manager.c
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#include "adc_manager.h"
#include "ads1115.h"

int16_t read_adc(ADCInput* adc) {
    if (adc->source == ADC_INTERNAL) {
    	printf("Internal measurement!\r\n");
        // Внутренний АЦП (HAL)
        // Предполагаем, что ADC-инстанс определён заранее, например:
        // hadc1, hadc2 и т.п. и выбирается по adc_id

        ADC_HandleTypeDef* hadc = NULL;
        switch (adc->adc_id) {
            case 0: hadc = &hadc1; break;
//             case 1: hadc = &hadc2; break; // Uncomment if ADC2 is in use!
            // добавь другие при необходимости
            default: printf("ADC not initialized!\r\n"); return 0;
        }
        ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.Channel = adc->channel;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;  // пример, нужно подобрать

		HAL_ADC_ConfigChannel(hadc, &sConfig);
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10);
        int16_t raw = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        adc->value = raw;
        return raw;
    } else if (adc->source == ADC_EXTERNAL) {
    	printf("External measurement!\r\n");
        if (adc->adc_id == 1) { // например, ADS1115
            if (adc->mode == ADC_DIFFERENTIAL) {
            	int16_t raw = ads1115_read_diff(ADS1115_ADDR, &hi2c1, adc->pos_channel, adc->neg_channel);
            	adc->value = raw;
                return ;
            } else {
            	int16_t raw = ads1115_read_single(ADS1115_ADDR, &hi2c1,adc->channel);
            	adc->value = raw;
                return raw;
            }
        }
    }

    return 0;
}

float get_voltage(ADCInput* adc){
	read_adc(adc);
	float voltage = 0.0f;
	if (adc->source == ADC_INTERNAL){
        float vref = 3.3f;
		float adc_max = 4095.0f;
		voltage = (adc->value / adc_max) * vref;
	    printf("Voltage x100 internal: %d\r\n", (int)(voltage*100));
	} else if (adc->source == ADC_EXTERNAL){
		float adc_max = 32768.0f;
	    voltage = adc->value * (ads1115_get_fsr(ADS1115_PGA) / adc_max);
	    printf("Voltage x100 ads1115: %d\r\n", (int)(voltage*100));
	}

	return voltage;
}
