# STM32 Laboratory Power Supply (LPS)

This repository contains the firmware for a multi-channel laboratory power supply based on the STM32F103C6Tx microcontroller. The project is developed using STM32CubeIDE and the STM32 HAL library. It provides a flexible and configurable solution for controlling multiple power outputs, with real-time monitoring and protection features.

## Table of Contents
1.  [Project Overview](#project-overview)
2.  [Hardware](#hardware)
3.  [Software Architecture](#software-architecture)
4.  [Features](#features)
5.  [Build and Deploy](#build-and-deploy)
6.  [Configuration](#configuration)
7.  [Usage](#usage)
8.  [Troubleshooting](#troubleshooting)
9.  [Limitations & Future Work](#limitations--future-work)
10. [Documentation](#documentation)
11. [License](#license)

---

## 1. Project Overview

The primary goal of this project is to create a versatile and robust power supply unit suitable for electronics lab work. It manages up to five independent power channels, each with its own set of sensors and controls. The system is designed to be highly configurable through code, allowing users to adapt it to various hardware setups.

The firmware provides a user interface via a 16x2 character LCD and several push-buttons, allowing for real-time monitoring and on-the-fly adjustments of parameters like PWM duty cycle and current limits.

## 2. Hardware

The firmware is designed for an STM32F103C6Tx MCU and a custom set of peripherals.

### Core Components
*   **Microcontroller**: `STM32F103C6Tx`
*   **Display**: 16x2 I2C LCD Module (based on HD44780 with a PCF8574 I/O expander).
*   **External ADC**: `ADS1115` 16-bit I2C ADC for precise current measurements.
*   **Sensors**:
    *   NTC Thermistors for temperature sensing on each channel (read by internal ADC).
    *   Current Shunt Resistor for current sensing (read by external ADS1115).
*   **User Input**: 6 push-buttons (5 for channels, 1 for global settings).
*   **Outputs**: Channels can be controlled by direct GPIO (for relays/MOSFETs) or PWM (for variable control).
*   **Cooling**: A PWM-controlled fan for thermal management.

### Default Pinout

| Function              | Pin(s)          | Details                                      |
|-----------------------|-----------------|----------------------------------------------|
| **Debug UART**        | `PA9`(TX), `PA10`(RX) | USART1, 115200 baud                          |
| **I2C Bus**           | `PB6`(SCL), `PB7`(SDA)  | I2C1, for LCD and ADS1115                    |
| **On-board LED**      | `PC13`          |                                              |
| **Fan PWM Output**    | `PA0`           | TIM2 Channel 1                               |
| **External Button**   | `PC15`          | "Settings" button                            |
|                       |                 |                                              |
| **Channel "In" (1)**  |                 |                                              |
| Output                | `PB15`          | GPIO                                         |
| Button Input          | `PA15`          |                                              |
| Temp Sensor           | `PB1`           | Internal ADC Channel 9                       |
| Current Sensor        | `ADS1115 A0-A1` | External ADC, Differential                 |
|                       |                 |                                              |
| **Channel "C1" (2)**  |                 |                                              |
| Output                | `PA8`           | PWM (TIM1 Channel 1)                         |
| Button Input          | `PB3`           |                                              |
| Temp Sensor           | `PB0`           | Internal ADC Channel 8                       |
|                       |                 |                                              |
| **Channel "C2" (3)**  |                 |                                              |
| Output                | `PB14`          | GPIO                                         |
| Button Input          | `PB4`           |                                              |
| Temp Sensor           | `PA7`           | Internal ADC Channel 7                       |
|                       |                 |                                              |
| **Channel "C3" (4)**  |                 |                                              |
| Output                | `PB13`          | GPIO                                         |
| Button Input          | `PB5`           |                                              |
| Temp Sensor           | `PA6`           | Internal ADC Channel 6                       |
|                       |                 |                                              |
| **Channel "C4" (5)**  |                 |                                              |
| Output                | `PB12`          | GPIO                                         |
| Button Input          | `PC14`          |                                              |
| Temp Sensor           | `PA5`           | Internal ADC Channel 5                       |

## 3. Software Architecture

The firmware is modular, separating application logic from low-level drivers.

*   **`Core/`**: Contains all application source code.
    *   **`main.c`**: Initializes the system, configures all hardware objects (channels, fans, buttons), and starts the main controller loop. This file serves as the primary hardware configuration file.
    *   **`controller_*.c`**: The core application logic.
        *   `controller.c`: Manages the main application loop, background tasks, and fan control.
        *   `controller_globals.c`: Defines all global state variables and UI-related enums.
        *   `controller_screen.c`: Handles all rendering to the 16x2 LCD, including different screens for main overview, channel details, and settings.
        *   `controller_buttons.c`: Implements the state machine for button inputs, dispatching actions based on the current UI state (e.g., short vs. long press).
        *   `controller_getset.c`: Provides helper functions to query system state, such as finding the channel with the highest temperature or current reading.
    *   **`sensor_reader.c`**: A dedicated module for reading all sensors, converting raw ADC values into physical units (Celsius, Amps), and checking against warning/shutdown thresholds.
    *   **`adc_manager.c`**: An abstraction layer that provides a unified interface (`get_voltage`) for reading from either the internal STM32 ADC or the external ADS1115.
    *   **Drivers & Helpers**:
        *   `ads1115.c`: Low-level driver for the external I2C ADC.
        *   `lcd_i2c.c`: Low-level driver for the I2C LCD.
        *   `buttons.c`: A generic, reusable button-handling library that detects short and long presses with debouncing.
        *   `power_channel.c`: Functions to activate, deactivate, or toggle a channel's output.
*   **`Drivers/`**: Contains STM32 HAL drivers and CMSIS files.
*   **`project_types.h`**: A crucial header that defines all major data structures for the system, such as `PowerChannel`, `TemperatureSensor`, `CurrentSensor`, `FanController`, and `Button`. This file effectively describes the object model of the power supply.
*   **`defines.h`**: Contains global constants and hardware-specific macros (like I2C addresses and compile-time feature flags).

## 4. Features

*   **Multi-Channel Control**: Manages up to 5 independent power channels.
*   **Flexible Outputs**: Supports both simple ON/OFF (GPIO) and variable (PWM) outputs per channel.
*   **Comprehensive Sensing**:
    *   **Temperature**: Per-channel temperature monitoring using NTC thermistors.
    *   **Current**: High-precision current monitoring via an external ADS1115 ADC.
*   **Two-Level Protection**: Each sensor has configurable **Warning** and **Shutdown** thresholds.
    *   If a shutdown threshold is breached, the corresponding channel is automatically deactivated.
    *   Alerts are displayed on the main screen.
*   **Automatic Fan Control**: Fan speed is automatically adjusted based on the highest temperature reading relative to its shutdown limit, ensuring efficient and quiet cooling.
*   **Interactive UI**:
    *   **Main Screen**: Provides a quick overview of all channel states (ON/OFF) and the most critical system alerts.
    *   **Channel Detail Screen**: Shows detailed metrics for a selected channel (current, temperature, output value).
    *   **Settings Menu**: A hierarchical menu for on-the-fly configuration of PWM duty cycle and current thresholds.
*   **Advanced Button Logic**: Supports both short and long presses for intuitive navigation and control.
*   **Debug Output**: Retargets `printf` to `USART1` for easy debugging with a UART-to-USB adapter.

## 5. Build and Deploy

### Prerequisites
*   **IDE**: [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (v1.10.0 or later recommended).
*   **Toolchain**: ARM GCC (comes bundled with STM32CubeIDE).
*   **Programmer**: An ST-LINK/V2 programmer/debugger.

### Build Steps
1.  Clone the repository.
2.  Open STM32CubeIDE and import the project using `File > Import... > General > Existing Projects into Workspace`.
3.  Select the project root directory.
4.  Build the project using `Project > Build All` (or `Ctrl+B`).

### Deployment
1.  Connect the ST-LINK programmer to your development board and computer.
2.  In STM32CubeIDE, right-click the project and select `Run As > STM32 MCU Application`.
3.  The IDE will automatically build and flash the firmware to the microcontroller.

## 6. Configuration

The firmware is designed to be configured primarily through code.

### Hardware Layout (`Core/Src/main.c`)
The most important configuration happens in `Core/Src/main.c`. The static arrays `channels`, `fans`, and `external_buttons` define the entire hardware setup. To adapt the firmware to your hardware, modify these structures.

**Example: Modifying a Power Channel**
```c
// In Core/Src/main.c

static PowerChannel *channels[MAX_CHANNELS] = {
    &(PowerChannel){
		.name = "5V", // Change channel name
        .temp_sensor_count = 1,
        .temp_sensors = {
            {
                .adc = {
                    .source = ADC_INTERNAL,
                    .mode = ADC_SINGLE_ENDED,
                    .channel = 9, // STM32 ADC Channel
                },
                .warning_threshold = 60,  // Set warning temp to 60°C
                .shutdown_threshold = 80, // Set shutdown temp to 80°C
				.nominal_resistance = 10000.0f, // NTC parameters
				.beta = 3950.0f,
				.series_resistor = 10000.0f,
            }
        },
        .current_sensor = NULL, // Disable current sensor for this channel
        .output = {
            .type = OUTPUT_GPIO, // Use a simple ON/OFF output
            .pin = { .port = Ch1_Out_GPIO_Port, .pin = Ch1_Out_Pin },
            .active_high = 1
        },
        .button = &(Button){ /* ... */ },
        .enabled = false
    },
    // ... other channels
};
```

### Global Constants (`Core/Inc/defines.h`)
This file contains system-wide constants that can be adjusted:
*   `DEBUG`: Set to `1` to enable `printf` debugging over UART.
*   `LCD_ADDR`: The I2C address of your LCD module.
*   `ADS1115_ADDR`: The I2C address of your external ADC.
*   `MAX_CHANNELS`, `MAX_FANS`, etc.: System limits.
*   `CONTINUOUS_MODE`, `DIFFERENTIAL_MODE`: Configuration for the ADS1115 driver.

## 7. Usage

The UI is navigated using the channel buttons and the external "Settings" button. In the settings menus, the first two channel buttons ("In" and "C1") act as **Decrease/Down** and **Increase/Up** respectively.

*   **Toggle Channel**: Short-press the corresponding channel button.
*   **View Channel Details**: Long-press the corresponding channel button. From the detail screen, a long-press on the same button returns to the main screen.
*   **Enter Settings**: Long-press the external button from the main screen.
*   **Navigate Settings**:
    *   Use the **Increase/Decrease** buttons to move the selection cursor `>`.
    *   Short-press the **Settings** button to confirm a selection or cycle through editable digits in a value.
    *   Long-press the **Settings** button to go back one menu level or exit the settings menu entirely.

## 8. Troubleshooting

*   **LCD is blank or shows garbage**:
    *   Verify the I2C address in `defines.h` matches your hardware. Note that the address is left-shifted by 1 (`0x27 << 1`).
    *   Check I2C wiring to `PB6` (SCL) and `PB7` (SDA) and ensure pull-up resistors are present.
*   **Incorrect Sensor Readings**:
    *   For temperature, double-check the `series_resistor`, `nominal_resistance`, and `beta` values for your NTC thermistor in `main.c`.
    *   For current, verify the `conversion_factor` (which is your shunt resistor's value in Ohms) in `main.c`.
*   **No Debug Output**:
    *   Ensure `DEBUG` is set to `1` in `defines.h`.
    *   Connect a UART-to-USB adapter to `PA9` (MCU TX) and `PA10` (MCU RX).
    *   Set your serial terminal to `115200` baud, 8-N-1.

## 9. Limitations & Future Work

### Known Limitations
*   **No Persistent Storage**: All settings (PWM values, current thresholds) are stored in RAM and will be lost on power-down.
*   **Limited UI**: The 16x2 LCD is functional but offers limited space for displaying information.
*   **Fixed Configuration**: The entire hardware layout is hard-coded in `main.c`. A more dynamic configuration method could be beneficial.

### Future Improvements
*   **Save Settings to Flash**: Implement functions to save and load configuration from the STM32's internal flash memory.
*   **Voltage Sensing**: The `VoltageSensor` structure exists but is not used. This feature could be fully implemented.
*   **PID Control**: For PWM outputs, implement a PID controller to regulate voltage or current instead of just setting a static duty cycle.
*   **UI Overhaul**: Upgrade to a graphical or OLED display for a more advanced user interface.
*   **Configuration File**: Move the hardware configuration from `main.c` into a dedicated `config.h` to make modifications cleaner and easier.

## 10. Documentation

The source code is commented in a Doxygen-compatible format. A `Doxyfile` is included in the project root. To generate full HTML documentation:
1.  Install [Doxygen](https://www.doxygen.nl/).
2.  Run `doxygen` in the project root directory.
3.  Open `docs/html/index.html` in a web browser.

## 11. License

*   The STM32 HAL library and other ST-provided code are licensed under ST's license agreement.
*   The user-written application code is provided under the **MIT License**. See the `LICENSE` file for details.