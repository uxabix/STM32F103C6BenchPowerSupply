/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LCD_ADDR (0x27 << 1)
#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#define LCD_DELAY_MS 5

#define ADS1115_ADDR (0x48 << 1)
#define ADS1115_REG_CONV   0x00
#define ADS1115_REG_CONFIG 0x01
#define CONTINUOUS_MODE 1  // Установи в 0 для single-shot
#define NON_DIFFERENTIAL_MODE 0

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

//
#define MAX_TEMP_SENSORS 2  // можно менять
#define MAX_CHANNELS 5

// Коэффициент пересчета АЦП ADS1115 -> ток (в А)
// Для 0.05 Ом и усиления по умолчанию: 1 bit = ~0.000125V / 0.05Ω = 2.5 mA
#define CURRENT_CONVERSION_FACTOR 0.000125f / 0.05f  // = 0.0025 (примерно)
#define TEMPERATURE_ERROR -30

typedef enum {
    ADC_INTERNAL,
    ADC_EXTERNAL
} ADCSource;

typedef enum {
    ADC_SINGLE_ENDED,
    ADC_DIFFERENTIAL
} ADCMode;

typedef enum {
    OUTPUT_NONE,
    OUTPUT_GPIO,
    OUTPUT_PWM
} OutputType;

typedef enum {
    BUTTON_NONE,
    BUTTON_SHORT_PRESS,
    BUTTON_LONG_PRESS,
    BUTTON_HOLD
} ButtonEvent;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIOPin;

typedef struct {
    ADCSource source;
    ADCMode mode;

    union {
        struct { // single-ended
            uint8_t channel;
        };
        struct { // differential
            uint8_t pos_channel;
            uint8_t neg_channel;
        };
    };

    uint8_t adc_id;              // 0 для внутреннего, 1+ — внешние
    float conversion_factor;     // АЦП -> физ.величина
    float value;                 // последнее значение
} ADCInput;

typedef struct {
    ADCInput adc;
    int8_t  warning_threshold;     // например 60.0
    int8_t  shutdown_threshold;    // например 85.0
    bool warning_triggered;
    bool shutdown_triggered;
    int8_t  last_value;

    float nominal_resistance;   // например, 10000.0
	float nominal_temperature;  // например, 298.15 (25°C в Кельвинах)
	float beta;                 // например, 3435.0
	float series_resistor;      // сопротивление в делителе, 10000.0
} TemperatureSensor;

typedef struct {
    ADCInput adc;
    float warning_threshold;
    float shutdown_threshold;
    bool warning_triggered;
    bool shutdown_triggered;
    float last_value;
} CurrentSensor;

typedef struct {
    ADCInput adc;
    float divider_ratio;         // например 11.0f для делителя 100k / 10k
    float last_value;
    float overvoltage_threshold;  // например, 15.0 В
    float undervoltage_threshold; // например, 1.0 В
    bool overvoltage_triggered;
    bool undervoltage_triggered;
} VoltageSensor;

typedef struct {
    GPIOPin pin;
    uint32_t debounce_ms;
    uint32_t long_press_ms;

    uint8_t state;               // текущее состояние
    uint32_t last_change_time;
    ButtonEvent event;
} Button;

typedef struct {
    OutputType type;
    GPIOPin pin;
    uint8_t pwm_channel;         // если PWM
    uint8_t active_high;
} OutputControl;

typedef struct {
    uint8_t id;
    TemperatureSensor temp_sensors[MAX_TEMP_SENSORS];
    uint8_t temp_sensor_count;

    CurrentSensor* current_sensor;   // может быть NULL
    VoltageSensor* voltage_sensor;   // может быть NULL

    OutputControl output;
    Button button;

    bool enabled;                 // включен/отключен
    bool in_warning_state;
    bool in_shutdown_state;
} PowerChannel;

typedef struct {
    OutputControl pwm;
    int8_t start_temp;
    int8_t max_temp;
    float current_speed; // 0.0 – 1.0
} FanController;


FanController fan;
PowerChannel channels[MAX_CHANNELS] = {
    // --- Канал 1 ---
    {
        .id = 1,
        .temp_sensor_count = 1,
        .temp_sensors = {
            {
                .adc = {
                    .source = ADC_INTERNAL,
                    .mode = ADC_SINGLE_ENDED,
                    .channel = 9,        // Ch1_Temp (например, канал 1 внутреннего АЦП)
                    .adc_id = 0,
                    .conversion_factor = 0.01f, // зависит от термистора и делителя
                },
                .warning_threshold = 50.0f,
                .shutdown_threshold = 70.0f,
				.nominal_resistance = 10000.0f,
				.nominal_temperature = 298.15f,
				.beta = 3435.0f,
				.series_resistor = 10000.0f,
            }
        },
        .current_sensor = &(CurrentSensor){
            .adc = {
                .source = ADC_EXTERNAL,
                .mode = ADC_DIFFERENTIAL,
                .pos_channel = 0,  // A0
                .neg_channel = 1,  // A1
                .adc_id = 1,       // ADS1115, например
                .conversion_factor = CURRENT_CONVERSION_FACTOR
            },
            .warning_threshold = 1.0f,
            .shutdown_threshold = 2.0f
        },
        .voltage_sensor = NULL, // не измеряет
        .output = {
            .type = OUTPUT_GPIO,
            .pin = { .port = Ch1_Out_GPIO_Port, .pin = Ch1_Out_Pin },
            .active_high = 1
        },
        .button = {
            .pin = { .port = Ch1_In_GPIO_Port, .pin = Ch1_In_Pin },
            .debounce_ms = 50,
            .long_press_ms = 1000,
            .state = 0,
            .last_change_time = 0,
            .event = BUTTON_NONE
        },
        .enabled = 1
    },

    // --- Канал 2–5 ---
    {
        .id = 2,
        .temp_sensor_count = 1,
        .temp_sensors = {
            {
                .adc = {
                    .source = ADC_INTERNAL,
                    .mode = ADC_SINGLE_ENDED,
                    .channel = 8,
                    .adc_id = 0,
                    .conversion_factor = 0.01f
                },
                .warning_threshold = 50,
                .shutdown_threshold = 70,
				.nominal_resistance = 10000.0f,
				.nominal_temperature = 298.15f,
				.beta = 3435.0f,
				.series_resistor = 10000.0f,
            }
        },
        .current_sensor = NULL,
        .voltage_sensor = NULL,
        .output = {
            .type = OUTPUT_GPIO,
            .pin = { .port = Ch2_Out_GPIO_Port, .pin = Ch2_Out_Pin },
            .active_high = 1
        },
        .button = {
            .pin = { .port = Ch2_In_GPIO_Port, .pin = Ch2_In_Pin },
            .debounce_ms = 50,
            .long_press_ms = 1000,
            .state = 0,
            .last_change_time = 0,
            .event = BUTTON_NONE
        },
        .enabled = 1
    },

    {
        .id = 3,
        .temp_sensor_count = 1,
        .temp_sensors = {
            {
                .adc = {
                    .source = ADC_INTERNAL,
                    .mode = ADC_SINGLE_ENDED,
                    .channel = 7,
                    .adc_id = 0,
                    .conversion_factor = 0.01f
                },
                .warning_threshold = 50,
                .shutdown_threshold = 70,
				.nominal_resistance = 10000.0f,
				.nominal_temperature = 298.15f,
				.beta = 3435.0f,
				.series_resistor = 10000.0f,
            }
        },
        .current_sensor = NULL,
        .voltage_sensor = NULL,
        .output = {
            .type = OUTPUT_GPIO,
            .pin = { .port = Ch3_Out_GPIO_Port, .pin = Ch3_Out_Pin },
            .active_high = 1
        },
        .button = {
            .pin = { .port = Ch3_In_GPIO_Port, .pin = Ch3_In_Pin },
            .debounce_ms = 50,
            .long_press_ms = 1000,
            .state = 0,
            .last_change_time = 0,
            .event = BUTTON_NONE
        },
        .enabled = 1
    },

    {
        .id = 4,
        .temp_sensor_count = 1,
        .temp_sensors = {
            {
                .adc = {
                    .source = ADC_INTERNAL,
                    .mode = ADC_SINGLE_ENDED,
                    .channel = 6,
                    .adc_id = 0,
                    .conversion_factor = 0.01f
                },
                .warning_threshold = 50,
                .shutdown_threshold = 70,
				.nominal_resistance = 10000.0f,
				.nominal_temperature = 298.15f,
				.beta = 3435.0f,
				.series_resistor = 10000.0f,
            }
        },
        .current_sensor = NULL,
        .voltage_sensor = NULL,
        .output = {
            .type = OUTPUT_GPIO,
            .pin = { .port = Ch4_Out_GPIO_Port, .pin = Ch4_Out_Pin },
            .active_high = 1
        },
        .button = {
            .pin = { .port = Ch4_In_GPIO_Port, .pin = Ch4_In_Pin },
            .debounce_ms = 50,
            .long_press_ms = 1000,
            .state = 0,
            .last_change_time = 0,
            .event = BUTTON_NONE
        },
        .enabled = 1
    },

    {
        .id = 5,
        .temp_sensor_count = 1,
        .temp_sensors = {
            {
                .adc = {
                    .source = ADC_INTERNAL,
                    .mode = ADC_SINGLE_ENDED,
                    .channel = 5,
                    .adc_id = 0,
                    .conversion_factor = 0.01f
                },
                .warning_threshold = 50,
                .shutdown_threshold = 70,
				.nominal_resistance = 10000.0f,
				.nominal_temperature = 298.15f,
				.beta = 3435.0f,
				.series_resistor = 10000.0f,
            }
        },
        .current_sensor = NULL,
        .voltage_sensor = NULL,
        .output = {
            .type = OUTPUT_GPIO,
            .pin = { .port = Ch5_Out_GPIO_Port, .pin = Ch5_Out_Pin },
            .active_high = 1
        },
        .button = {
            .pin = { .port = Ch5_In_GPIO_Port, .pin = Ch5_In_Pin },
            .debounce_ms = 50,
            .long_press_ms = 1000,
            .state = 0,
            .last_change_time = 0,
            .event = BUTTON_NONE
        },
        .enabled = 1
    }
};

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
void ads1115_init_continuous(uint8_t pos_channel, uint8_t neg_channel) {
#if CONTINUOUS_MODE
    uint16_t config = 0;
    config |= get_mux_bits(pos_channel, neg_channel);
    config |= (0b010 << 9);  // PGA = ±0.256V
    config |= (0 << 8);      // Continuous mode
    config |= (0b100 << 5);  // 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, buf, 3, HAL_MAX_DELAY);
#endif
}

void ads1115_init_single_continuous(uint8_t channel) {
#if CONTINUOUS_MODE
    if (channel > 3) return;

    uint16_t config = 0;
    config |= (0b1000 + channel) << 12;  // MUX AINx vs GND
    config |= (0b010 << 9);  // PGA = ±0.256V
    config |= (0 << 8);      // Continuous mode
    config |= (0b100 << 5);  // 128 SPS
    config |= 3;             // Disable comparator

    uint8_t buf[3];
    buf[0] = ADS1115_REG_CONFIG;
    buf[1] = (uint8_t)(config >> 8);
    buf[2] = (uint8_t)(config & 0xFF);

    HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, buf, 3, HAL_MAX_DELAY);
#endif
}

float ads1115_read_diff(uint8_t pos_channel, uint8_t neg_channel){
#if !CONTINUOUS_MODE
    // Одноразовое измерение (single-shot)
    uint16_t config = 0;
    config |= 1 << 15;  // Start single conversion
    config |= get_mux_bits(pos_channel, neg_channel);
    config |= (0b010 << 9);  // PGA = ±0.256V
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
    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return -9999.0f;
    if (HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return -9999.0f;

    int16_t raw = (data[0] << 8) | data[1];
    float voltage = raw * 0.0000078125f;  // Для ±0.256V

    return voltage;
}

float ads1115_read_single(uint8_t channel){
	if (channel > 3) return -9999.0f;

#if !CONTINUOUS_MODE
	uint16_t config = 0;
	config |= (1 << 15);  // Start single-shot
	config |= (0b1000 + channel) << 12;  // MUX AINx vs GND
	config |= (0b010 << 9);  // PGA = ±0.256V
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

	if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
		return -9999.0f;
	if (HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
		return -9999.0f;

	int16_t raw = (data[0] << 8) | data[1];
	float voltage = raw * 0.0000078125f;  // при ±0.256V

	return voltage;
}

float read_adc(const ADCInput* adc) {
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
            default: printf("ADC not initialized!\r\n"); return 0.0f;
        }
        ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.Channel = adc->channel;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;  // пример, нужно подобрать

		HAL_ADC_ConfigChannel(hadc, &sConfig);
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10);
        uint32_t raw = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return (float)raw;
    } else if (adc->source == ADC_EXTERNAL) {
    	printf("External measurement!\r\n");
        if (adc->adc_id == 1) { // например, ADS1115
            if (adc->mode == ADC_DIFFERENTIAL) {
                return ads1115_read_diff(adc->pos_channel, adc->neg_channel);
            } else {
                return ads1115_read_single(adc->channel);
            }
        }
    }

    return 0.0f;
}

/**
 * Вычисляет температуру в градусах Цельсия по сопротивлению термистора.
 *
 * @param r_therm Сопротивление термистора в Омах
 * @param nominal_resistance Номинальное сопротивление (обычно 10k) при 25°C
 * @param nominal_temperature Номинальная температура в Кельвинах (обычно 298.15)
 * @param beta Параметр β термистора (например, 3435)
 * @return Температура в °C
 */
float thermistor_to_celsius(float r_therm, float nominal_resistance,
                            float nominal_temperature, float beta) {
    if (r_therm <= 0.0f) return -273.15f;  // ошибка: невозможное сопротивление

    float inv_T = (1.0f / nominal_temperature) +
                  (1.0f / beta) * logf(r_therm / nominal_resistance);

    float temp_K = 1.0f / inv_T;
    return temp_K - 273.15f;
}

PowerChannel* update_all_temperatures(void) {
	PowerChannel* maxTempChannel = &channels[0];
	float maxTempCoefficient = 0;
	printf("Temp check started!\r\n");
    for (int ch = 0; ch < MAX_CHANNELS; ++ch) {
    	printf("Channel %d", ch);
        PowerChannel* channel = &channels[ch];

        for (int i = 0; i < channel->temp_sensor_count; ++i) {
        	printf("Sensor %d\r\n", i);
            TemperatureSensor* sensor = &channel->temp_sensors[i];
            float raw = read_adc(&sensor->adc);
            printf("Raw: %d\r\n", (int)raw);
            float vref = 3.3f;
			float adc_max = 4095.0f;
			float voltage = (raw / adc_max) * vref;

			// Делитель напряжения: Vs = Vcc * R_therm / (R_therm + R_fixed)
			float r_therm = sensor->series_resistor * voltage / (vref - voltage);

			float temp_c = thermistor_to_celsius(
				    r_therm,
				    sensor->nominal_resistance,
				    sensor->nominal_temperature,
				    sensor->beta
				);

            sensor->adc.value = raw; // сохраняем сырое значение
            sensor->last_value = temp_c;
            sensor->warning_triggered = (temp_c > sensor->warning_threshold);
            sensor->shutdown_triggered = (temp_c < sensor->shutdown_threshold);
            printf("Temp: %d\r\n", (int)temp_c);
			if (temp_c / sensor->shutdown_threshold > maxTempCoefficient){
				maxTempChannel = &channels[ch];
				maxTempCoefficient = temp_c / sensor->shutdown_threshold;
			}

            if (sensor->shutdown_triggered) {
                channel->enabled = 0;
                channel->in_shutdown_state = 1;
            } else if (sensor->warning_triggered || temp_c < TEMPERATURE_ERROR) {
                channel->in_warning_state = 1;
            } else {
                channel->in_warning_state = 0;
                channel->in_shutdown_state = 0;
            }
        }
    }

    return maxTempChannel;
}

void update_all_currents_and_voltages(void) {
    for (int ch = 0; ch < MAX_CHANNELS; ++ch) {
        PowerChannel* channel = &channels[ch];

        // --- Ток ---
        if (channel->current_sensor != NULL) {
            CurrentSensor* cs = channel->current_sensor;
            float raw = read_adc(&cs->adc);
            float current = raw * cs->adc.conversion_factor;

            cs->adc.value = raw;
            cs->last_value = current;
            cs->warning_triggered = (current > cs->warning_threshold);
            cs->shutdown_triggered = (current < cs->shutdown_threshold);
            if (cs->shutdown_triggered) {
                channel->enabled = 0;
                channel->in_shutdown_state = 1;
            } else if (cs->warning_triggered) {
                channel->in_warning_state = 1;
            } else {
                channel->in_warning_state = 0;
                channel->in_shutdown_state = 0;
            }
        }

        // --- Напряжение ---
        if (channel->voltage_sensor != NULL) {
            VoltageSensor* vs = channel->voltage_sensor;
            float raw = read_adc(&vs->adc);
            float voltage = raw * vs->adc.conversion_factor / vs->divider_ratio;

            vs->adc.value = raw;
            vs->last_value = voltage;

            // Можешь добавить свои пороги или реакции здесь

            vs->overvoltage_triggered = (voltage > vs->overvoltage_threshold);
            vs->undervoltage_triggered = (voltage < vs->undervoltage_threshold);
            if (vs->overvoltage_triggered){
            	channel->enabled = 0;
            	channel->in_shutdown_state = 1;
            } else if (vs->undervoltage_triggered){
            	channel->in_warning_state = 1;
			} else {
                channel->in_warning_state = 0;
                channel->in_shutdown_state = 0;
            }
        }
    }
}



// i2c

void I2C_Scan(void) {
    printf("Scanning I2C bus...\r\n");
    HAL_Delay(100);

    for (uint8_t address = 1; address < 127; address++) {
        // <<1: преобразуем 7-битный адрес в 8-битный для HAL
        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, 10) == HAL_OK) {
            printf("Device found at 0x%02X\r\n", address);
        }
    }

    printf("Scan complete.\r\n");
}

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
                                   uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1,
                                    HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr,
                                  sizeof(data_arr), HAL_MAX_DELAY);
//    HAL_Delay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(lcd_addr, 0b00110000);
    HAL_Delay(LCD_DELAY_MS);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd_addr, 0b00000010);
    HAL_Delay(LCD_DELAY_MS);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, 0b00001100);
    HAL_Delay(LCD_DELAY_MS);
    // clear display (optional here)
    LCD_SendCommand(lcd_addr, 0b00000001);
    HAL_Delay(LCD_DELAY_MS);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting up...\r\n");
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  LCD_Init(LCD_ADDR);
  ads1115_init_continuous(0, 1);  // AIN0-AIN1
#ifdef NON_DIFFERENTIAL_MODE
  ads1115_init_single_continuous(1); // Example
#endif
  // set address to 0x00
  LCD_SendCommand(LCD_ADDR, 0b10000000);
  LCD_SendString(LCD_ADDR, " Using 1602 LCD");
  // set address to 0x40
  LCD_SendCommand(LCD_ADDR, 0b11000000);
  LCD_SendString(LCD_ADDR, "  over I2C bus");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PowerChannel* maxTempChannel = &channels[0];
  char str[16];
  while (1)
  {
	LCD_SendCommand(LCD_ADDR, 0b00000001);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	maxTempChannel = update_all_temperatures();
	snprintf(str, sizeof(str), "Ch%d = %d", maxTempChannel->id, (int)maxTempChannel->temp_sensors[0].last_value);
    LCD_SendCommand(LCD_ADDR, 0b10000000);
    LCD_SendString(LCD_ADDR, str);
	HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Ch5_Out_Pin|Ch4_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ch2_Out_GPIO_Port, Ch2_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch5_In_Pin */
  GPIO_InitStruct.Pin = Ch5_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch5_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch5_Out_Pin Ch4_Out_Pin */
  GPIO_InitStruct.Pin = Ch5_Out_Pin|Ch4_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch3_Out_Pin */
  GPIO_InitStruct.Pin = Ch3_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(Ch3_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch2_Out_Pin */
  GPIO_InitStruct.Pin = Ch2_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ch2_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch1_In_Pin */
  GPIO_InitStruct.Pin = Ch1_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch1_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch2_In_Pin Ch3_In_Pin Ch4_In_Pin */
  GPIO_InitStruct.Pin = Ch2_In_Pin|Ch3_In_Pin|Ch4_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
