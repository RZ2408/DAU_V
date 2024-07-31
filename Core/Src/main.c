/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <string.h> // Include the string.h header for strlen function
#include <stdbool.h>  //
#include <stdio.h>
#include "i2c_max11611.h"


extern UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_adc;

uint32_t adc_values[2];
uint32_t max11611_adc_values[4];

#define ADC_RESOLUTION 4096
#define VREF 3.3
#define MAX11611_ADDRESS 0x35 // 7-bit address

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define POWER_DOWN_DURATION 15000 // 15 seconds
#define LOW_DURATION_THRESHOLD 2000 // 2 seconds in milliseconds
#define VOLTAGE_CHECK_INTERVAL 15000 // 15 seconds in milliseconds
#define BLINK_FREQUENCY 2000 // 2 kHz




extern uint16_t adc8_value;  // These should be updated by your ADC reading functions
extern uint16_t adc9_value;

uint16_t adc8_value = 0;
uint16_t adc9_value = 0;

uint32_t reset_timer_start = 0;
uint8_t core_rst_in_active = 0;
uint8_t force_low_enable = 0;
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
void sys_isense_adc8(void);
void pwrin_vsense_adc9(void);


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE BEGIN PFP */
void sys_isense_adc8(void);
void pwrin_vsense_adc9(void);

void handle_pin_transitions(void);
uint8_t Check_Power_Good_Pins(void);


bool check_adc_pwr(void);
bool check_max11611_voltage_conditions(uint8_t devAddr);
bool check_power_good_pins_conditions(void);
bool Power_Conditioning_Check(uint8_t devAddr);


void display_menu() {
    char menu[] = "Menu:\r\n1. Test Max11611 Values\r\n2. Reset uC\r\n3. Read Vsense\r\n4. Read Isense\r\n5. Check ADC MCU\r\n6. Check Max11611 Voltage Conditions\r\n7. Check Power Good Pins Status\r\n8. Power Conditioning Check\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)menu, strlen(menu), HAL_MAX_DELAY);
}

void handle_menu_selection(uint8_t selection) {
    switch (selection) {
        case '1':
            i2c_readMax11611Adc(MAX11611_ADDRESS);
            break;
        case '2': {
            char reset_msg[] = "Resetting uC ...\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)reset_msg, strlen(reset_msg), HAL_MAX_DELAY);

            // Disable power
            HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_RESET);

            // Wait for 10 seconds
            HAL_Delay(10000);

            // Reset microcontroller
            NVIC_SystemReset();
            break;
        }
        case '3':
            pwrin_vsense_adc9();
            break;
        case '4':
            sys_isense_adc8();
            break;

        case '5': {
                    bool result = check_adc_pwr();
                    if (result) {
                        char msg[] = "Voltage conditions met .\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    } else {
                        char msg[] = "Voltage conditions not met .\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    }
                    break;
                    }
        case '6': {
                    bool voltage_ok = check_max11611_voltage_conditions(MAX11611_ADDRESS);
                    if (voltage_ok) {
                        char msg[] = "Max11611 Voltage conditions met (all channels >= 1,4V).\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    } else {
                        char msg[] = "Max11611 Voltage conditions not met (at least one channel < 1.4V).\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    }
                    break;
                }
        case '7': {
                    // New case for checking power good pins status
                    bool status_ok = check_power_good_pins_conditions();
                    if (status_ok) {
                        char msg[] = "All Digital In are High.\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    } else {
                        char msg[] = "One or more Digital In are low.\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    }
                    break;
        }


        case '8': {
                    // New case for Power Conditioning Check
                    bool power_condition_ok = Power_Conditioning_Check(MAX11611_ADDRESS);
                    if (power_condition_ok) {
                        char msg[] = "All Power conditions are OK.\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    } else {
                        char msg[] = "One or more Power conditions failed. Actions taken.\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    }
                    break;    }
    }


    // Display menu again after handling selection
    display_menu();
}



// Function to check the specific voltage conditions



void sys_isense_adc8(void) {
	uint32_t adc8_value = adc_values[0];
	float voltage8 = (float)adc8_value * VREF / ADC_RESOLUTION;

	// Convert voltage to current using the given formula
	float I_Sense = voltage8 / 0.169;

	char buffer[100];
	snprintf(buffer, sizeof(buffer), "Isense Value: %lu, Voltage: %.2fV, Current: %.2fA\r\n", adc8_value, voltage8, I_Sense);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

}

void pwrin_vsense_adc9(void) {
    uint32_t adc9_value = adc_values[1];
    float voltage9 = (float)adc9_value * VREF / ADC_RESOLUTION;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Vsense Value: %lu, Voltage: %.2fV\r\n", adc9_value, voltage9);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}


// Power Conditioning
bool check_adc_pwr(void) {
    // Call the functions to update adc8_value and adc9_value
    pwrin_vsense_adc9();
    sys_isense_adc8();

    // Calculate the voltages
    uint32_t adc8_value = adc_values[0];
    float voltage8 = (float)adc8_value * VREF / ADC_RESOLUTION;
    uint32_t adc9_value = adc_values[1];
    float voltage9 = (float)adc9_value * VREF / ADC_RESOLUTION;

    // Convert voltage to current using the given formula
    float I_Sense = voltage8 / 0.169;

    // Print the voltages and current for debugging purposes
    char buffer[150];
    snprintf(buffer, sizeof(buffer), "Debug - Isense Voltage: %.2fV, Vsense Voltage: %.2fV, Current: %.2fA\r\n", voltage8, voltage9, I_Sense);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Check if the voltages and current meet the specified conditions
    return (voltage9 >= 0.6f && voltage9 <= 2.2f && I_Sense >= 0.6f && I_Sense <= 8.5f);
}


bool check_max11611_voltage_conditions(uint8_t devAddr) {
    uint32_t volt[4]; // Array to store the voltages of the 4 channels
    float voltage[4];
    char buffer[200]; // Adjust buffer size to accommodate all channels
    const float VREF_CONST = 3.3f; // Adjust VREF as per your reference voltage
    const uint32_t ADC_RESOLUTION_CONST = 4096; // Adjust based on your ADC resolution

    // Read the ADC values for the 4 channels
    i2c_readMax11611Adc(devAddr); // Call the original function

    // Copy the values to local array
    for (int i = 0; i < 4; i++) {
        volt[i] = max11611_adc_values[i];
    }

    // Calculate the voltages
    for (int i = 0; i < 4; i++) {
        voltage[i] = (float)volt[i] * VREF_CONST / ADC_RESOLUTION_CONST;
    }

    // Print the voltages for debugging purposes
    snprintf(buffer, sizeof(buffer), "Debug - AIN0 Voltage: %.2fV, AIN1 Voltage: %.2fV, AIN2 Voltage: %.2fV, AIN3 Voltage: %.2fV\r\n",
             voltage[0], voltage[1], voltage[2], voltage[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Check if the voltages are between 1.4V and 2.5V for all channels
    return (voltage[0] >= 1.4f && voltage[0] <= 2.5f &&
            voltage[1] >= 1.4f && voltage[1] <= 2.5f &&
            voltage[2] >= 1.4f && voltage[2] <= 2.5f &&
            voltage[3] >= 1.4f && voltage[3] <= 2.5f);
}

bool check_power_good_pins_conditions(void) {
    char buffer[200];

    // Check GPIO pin states
    GPIO_PinState pin_states[3] = {
        HAL_GPIO_ReadPin(CORE_PWR_CONN_PG_GPIO_Port, CORE_PWR_CONN_PG_Pin),
        HAL_GPIO_ReadPin(V3_PG_GPIO_Port, V3_PG_Pin),
        HAL_GPIO_ReadPin(V3STB_PG_GPIO_Port, V3STB_PG_Pin)
    };

    snprintf(buffer, sizeof(buffer),
        "Debug - Power Good Pins States:\r\n"
        "CORE_PWR_CONN_PG: %s\r\n"
        "V3_PG: %s\r\n"
        "V3STB_PG: %s\r\n",
        pin_states[0] == GPIO_PIN_SET ? "HIGH" : "LOW",
        pin_states[1] == GPIO_PIN_SET ? "HIGH" : "LOW",
        pin_states[2] == GPIO_PIN_SET ? "HIGH" : "LOW"
    );
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Check if all pins are HIGH
    for (int i = 0; i < 3; i++) {
        if (pin_states[i] != GPIO_PIN_SET) {
            HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_RESET);
            HAL_Delay(1500);
            HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_SET);
            return false;
        }
    }
    return true;
}


bool Power_Conditioning_Check(uint8_t devAddr) {
    // Check the conditions
    bool voltage_ok = check_adc_pwr();
    bool max11611_voltage_ok = check_max11611_voltage_conditions(devAddr);
    bool power_good_pins_ok = check_power_good_pins_conditions();

    // If any condition is false, take action
    if (!voltage_ok || !max11611_voltage_ok || !power_good_pins_ok) {
        // Print debug information about which condition failed
        char buffer[200];
        snprintf(buffer, sizeof(buffer),
                 "Condition Check Results:\r\n"
                 "Voltage Condition: %s\r\n"
                 "MAX11611 Voltage Condition: %s\r\n"
                 "Power Good Pins Condition: %s\r\n",
                 voltage_ok ? "OK" : "FAIL",
                 max11611_voltage_ok ? "OK" : "FAIL",
                 power_good_pins_ok ? "OK" : "FAIL");
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        // Disable power
        HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_RESET);

        // Wait for 10 seconds
        HAL_Delay(10000);

        // Reset microcontroller
        NVIC_SystemReset();

        return false;
    }

    return true;
}



uint32_t read_adc_channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure ADC Channel
    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    // Reconfigure ADC channel
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Error configuring channel %lu\r\n", channel);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        Error_Handler();
    }

    // Start ADC conversion
    if (HAL_ADC_Start(&hadc) != HAL_OK) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Error starting ADC for channel %lu\r\n", channel);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        Error_Handler();
    }

    // Poll for conversion completion
    if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) != HAL_OK) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Error polling ADC for channel %lu\r\n", channel);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        Error_Handler();
    }

    // Get the ADC value
    uint32_t adc_value = HAL_ADC_GetValue(&hadc);

    // Stop ADC conversion
    if (HAL_ADC_Stop(&hadc) != HAL_OK) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Error stopping ADC for channel %lu\r\n", channel);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        Error_Handler();
    }

    // Print debug message
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), "Channel %lu ADC value: %lu\r\n", channel, adc_value);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);

    return adc_value;
}

    // Rest of the function (optional)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Check if either pin goes from high to low
    if (HAL_GPIO_ReadPin(MCUIN_RESET_GPIO_Port, MCUIN_RESET_Pin) == GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(CORE_RST_OUT_GPIO_Port, CORE_RST_OUT_Pin) == GPIO_PIN_RESET) {

        if (GPIO_Pin == MCUIN_RESET_Pin || GPIO_Pin == CORE_RST_OUT_Pin) {
            if (!core_rst_in_active) {
                reset_timer_start = HAL_GetTick();
                core_rst_in_active = true;

                // Set CORE_RST_IN_Pin high
                HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_SET);
            }
        }
    }
    // Check if either pin goes from low to high
    else if (HAL_GPIO_ReadPin(MCUIN_RESET_GPIO_Port, MCUIN_RESET_Pin) == GPIO_PIN_SET &&
             HAL_GPIO_ReadPin(CORE_RST_OUT_GPIO_Port, CORE_RST_OUT_Pin) == GPIO_PIN_SET) {

        // Reset CORE_RST_IN_Pin to low
        HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_RESET);
        core_rst_in_active = false;
    }
}

// Function to handle pin state transitions
void handle_pin_transitions(void) {
    // Check if the pin has been low for at least 2 seconds continuously
    if (core_rst_in_active && HAL_GetTick() - reset_timer_start >= 2000) {
        // Reset CORE_RST_IN_Pin to low
        HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_RESET);
        core_rst_in_active = false;
    }
}
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_values, 2);

  /* USER CODE END 2 */
  // Display the menu at startup
              display_menu();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Power_Conditioning_Check(MAX11611_ADDRESS);


	  uint8_t received_char;
	  	          if (HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY) == HAL_OK) {
	  	              handle_menu_selection(received_char);
	  	          }
	  	      handle_pin_transitions();

	  	      // Handle the reset pin if the flag is set
	  	      if (core_rst_in_active && HAL_GetTick() - reset_timer_start >= LOW_DURATION_THRESHOLD) {
	  	          // Reset CORE_RST_IN_Pin to low
	  	          HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_RESET);
	  	          core_rst_in_active = 0;
	  	      }




	    }
	    /* USER CODE END 3 */





  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MAIN_PWR_ENABLE_Pin */
  GPIO_InitStruct.Pin = MAIN_PWR_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_PWR_ENABLE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
/* EXTI line interrupt callback */
void EXTI4_15_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(MCUIN_RESET_Pin);
    HAL_GPIO_EXTI_IRQHandler(CORE_RST_OUT_Pin);
}
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
