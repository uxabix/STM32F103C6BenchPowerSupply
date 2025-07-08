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
#include <stddef.h>

#include "project_types.h"
#include "lcd_i2c.h"
#include "adc_manager.h"
#include "power_channel.h"
#include "sensor_reader.h"

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
TIM_HandleTypeDef htim2;

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
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

//
#define SCREEN_UPDATE_PERIOD 1000 // ms
uint32_t lastScreenUpdate = 0;

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
                .conversion_factor = 0.05,
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
            .type = OUTPUT_PWM,
			.pwm_timer = &htim1,
			.pwm_channel = TIM_CHANNEL_1,
            .active_high = 1,
			.pwm_inversed = false,
			.pwm_last_value = htim1.Init.Period,
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting up...\r\n");
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  LCD_Init(&hi2c1, LCD_ADDR);
  ads1115_init_continuous(ADS1115_ADDR, &hi2c1, 0, 1);  // AIN0-AIN1
#if NON_DIFFERENTIAL_MODE
  ads1115_init_single_continuous(1); // Example
#endif
  // set address to 0x00
  LCD_SendCommand(LCD_ADDR, 0b10000000);
  LCD_SendString(LCD_ADDR, " Using 1602 LCD");
  // set address to 0x40
  LCD_SendCommand(LCD_ADDR, 0b11000000);
  LCD_SendString(LCD_ADDR, "  over I2C bus");
  HAL_Delay(1000);


  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  uint32_t duty = htim2.Init.Period * 0.5; // 50% скважность
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
  duty = htim1.Init.Period * 0.5; // 50% скважность
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PowerChannel* maxTempChannel = NULL;
  PowerChannel* maxCurrentChannel = NULL;
  PowerChannel* maxVoltageChannel = NULL;
  char str[16];
  char amps[8];
  char volts[8];
  while (1)
  {
	HAL_Delay(1000);
	maxTempChannel = update_all_temperatures();
	update_all_currents_and_voltages();

	if (HAL_GetTick() - lastScreenUpdate >= SCREEN_UPDATE_PERIOD){
		lastScreenUpdate = HAL_GetTick();
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		// Clear screen
		LCD_Clear(LCD_ADDR);
		if (maxTempChannel != NULL){
			snprintf(str, sizeof(str), "Ch%d=%d", maxTempChannel->id, (int)maxTempChannel->temp_sensors[0].last_value);
			LCD_SetFirstLine(LCD_ADDR);
		    LCD_SendString(LCD_ADDR, str);
		} else {
			strcpy(str, "No T");
			LCD_SetFirstLine(LCD_ADDR);
			LCD_SendString(LCD_ADDR, str);
		}
		memset(str, 0, sizeof(str));
		memset(amps, 0, sizeof(amps));
		memset(volts, 0, sizeof(volts));
	    maxCurrentChannel = get_channel_with_max_current();
	    if (maxCurrentChannel != NULL){
			snprintf(amps, sizeof(str), "Ch%d=%dA ", maxCurrentChannel->id, (int)maxTempChannel->current_sensor->last_value);
		} else {
			strcpy(amps, "No A ");
		}

	    maxVoltageChannel = get_channel_with_max_voltage();
	    if (maxVoltageChannel != NULL){
			snprintf(volts, sizeof(str), "Ch%d=%dV", maxVoltageChannel->id, (int)maxVoltageChannel->voltage_sensor->last_value);
		} else{
			strcpy(volts, "No V");
		}
	    strcat(str, amps);
	    strcat(str, volts);
	    LCD_SetSecondLine(LCD_ADDR);
		LCD_SendString(LCD_ADDR, str);
		duty = htim2.Init.Period * 1; // 50% скважность
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
		duty = htim1.Init.Period * 0.1; // 50% скважность
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

	}

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOB, Ch5_Out_Pin|Ch4_Out_Pin|Ch1_Out_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : Ch5_Out_Pin Ch4_Out_Pin Ch1_Out_Pin */
  GPIO_InitStruct.Pin = Ch5_Out_Pin|Ch4_Out_Pin|Ch1_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch3_Out_Pin */
  GPIO_InitStruct.Pin = Ch3_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(Ch3_Out_GPIO_Port, &GPIO_InitStruct);

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
