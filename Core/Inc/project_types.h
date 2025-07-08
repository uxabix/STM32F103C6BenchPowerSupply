/*
 * project_types.h
 *
 *  Created on: Jul 7, 2025
 *      Author: kiril
 */

#ifndef INC_PROJECT_TYPES_H_
#define INC_PROJECT_TYPES_H_

#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include "defines.h"

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
    float conversion_factor;     // For ammeter shunt resistance
    int16_t value;                 // последнее значение
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

#endif /* INC_PROJECT_TYPES_H_ */
