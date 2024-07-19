/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h> // Include the string.h header for strlen function
#include <stdbool.h>  // Add this line
#include <stdio.h>
#include "i2c_max11611.h"
extern UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_adc;

uint32_t adc_values[2];

#define ADC_RESOLUTION 4096
#define VREF 3.3




#define MAX11611_ADDRESS 0x35 // 7-bit address

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define POWER_DOWN_DURATION 15000 // 15 seconds
#define LOW_DURATION_THRESHOLD 2000 // 2 seconds in milliseconds
#define VOLTAGE_CHECK_INTERVAL 15000 // 15 seconds in milliseconds
#define BLINK_FREQUENCY 2000 // 2 kHz
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Global Variables */
/* Global Variables */
/* Flag to track whether to force MAIN_PWR_ENABLE low */

uint32_t reset_timer_start = 0;
uint8_t core_rst_in_active = 0;
uint8_t force_low_enable = 0;




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void sys_isense_adc8(void);
void pwrin_vsense_adc9(void);

void handle_pin_transitions(void);
uint8_t Check_Power_Good_Pins(void);

uint32_t read_adc_channel(uint32_t channel);

float AdcRead(uint32_t channel, uint8_t numAverage);




/* USER CODE BEGIN PFP */


void display_menu() {
    char menu[] = "Menu:\r\n1. Test Max11611 Values\r\n2. Reset uC\r\n3. Read Vsense\r\n4. Read Isense\r\n5. Check Power Good Pins\r\n";
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
            uint8_t power_good = Check_Power_Good_Pins();
            if (power_good) {
                char msg[] = "Power is good.\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            } else {
                char msg[] = "Power is NOT good.\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            break;
        }
        default:
            break;
    }

    // Display menu again after handling selection
    display_menu();
}



void sys_isense_adc8(void) {
    uint32_t adc8_value = adc_values[0];
    float voltage = (float)adc8_value * VREF / ADC_RESOLUTION;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Isense Value: %lu, Voltage: %.2fV\r\n", adc8_value, voltage);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void pwrin_vsense_adc9(void) {
    uint32_t adc9_value = adc_values[1];
    float voltage = (float)adc9_value * VREF / ADC_RESOLUTION;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Vsense Value: %lu, Voltage: %.2fV\r\n", adc9_value, voltage);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
uint8_t Check_Power_Good_Pins(void) {
    // Check GPIO pin states
    if (HAL_GPIO_ReadPin(CORE_3V3_5V_PG_GPIO_Port, CORE_3V3_5V_PG_Pin) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(CORE_PWR_ISO_PG_GPIO_Port, CORE_PWR_ISO_PG_Pin) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(CORE_PWR_CONN_PG_GPIO_Port, CORE_PWR_CONN_PG_Pin) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(V3_PG_GPIO_Port, V3_PG_Pin) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(V3STB_PG_GPIO_Port, V3STB_PG_Pin) != GPIO_PIN_SET) {
        // Not all pins are high
        HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_RESET);
        HAL_Delay(1500);
        HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_SET);
        return 0;
    }

    // Read ADC values from channels 1 to 4
    uint32_t volt[4];
    for (int i = 0; i < 4; i++) {
        uint8_t response[2];
        uint16_t val;
        HAL_StatusTypeDef status;

        // Configure which channels to read
        uint8_t config = 0b01100001 | (i << 1); // sel[0:2] = read channel as in CS[0:3], single ended
        status = HAL_I2C_Master_Transmit(&hi2c2, (MAX11611_ADDRESS << 1), &config, sizeof(config), HAL_MAX_DELAY);
        if (status != HAL_OK) {
            char error_msg[50];
            snprintf(error_msg, sizeof(error_msg), "I2C Tx Error on ch %d of MAX11611.\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
            return 0; // Error in I2C transmission
        }

        status = HAL_I2C_Master_Receive(&hi2c2, (MAX11611_ADDRESS << 1), response, sizeof(response), HAL_MAX_DELAY);
        if (status != HAL_OK) {
            char error_msg[50];
            snprintf(error_msg, sizeof(error_msg), "I2C Rx Error on ch %d of MAX11611.\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
            return 0; // Error in I2C reception
        }

        val = ((response[0] & 0x03) << 8) | response[1];
        volt[i] = val * 2;  // Adjust this calculation based on your voltage conversion needs

        // Check if any ADC value is greater than 1900
        if (volt[i] > 1900) {
            HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_SET);
            return 1; // All conditions are met
        }
    }

    // If any condition fails
    HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_Delay(1500);
    HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_SET);
    return 0; // Not all conditions are met
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

  uint8_t Check_Power_Good_Pins();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  // Set MAIN_PWR_ENABLE high for 1 second at system start


  // Start ADC conversion with DMA
     if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_values, 2) != HAL_OK) {
         Error_Handler();
     }
     // Display the menu at startup
         display_menu();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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


/**
  * @brief System Clock Configuration
  *
  *
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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



static void MX_ADC_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(&hadc) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_9;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_adc) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&hadc, DMA_Handle, hdma_adc);
}

  /** Configure for the selected ADC regular channel to be converted.
  */
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_PWR_ENABLE_GPIO_Port, MAIN_PWR_ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PWR_LED_ON_Pin|FAULT_LED_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CORE_RST_IN_GPIO_Port, CORE_RST_IN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MCUIN_RESET_Pin */
  GPIO_InitStruct.Pin = MCUIN_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCUIN_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAIN_PWR_ENABLE_Pin */
  GPIO_InitStruct.Pin = MAIN_PWR_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_PWR_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_LED_ON_Pin FAULT_LED_ON_Pin */
  GPIO_InitStruct.Pin = PWR_LED_ON_Pin|FAULT_LED_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CORE_RST_OUT_Pin */
  GPIO_InitStruct.Pin = CORE_RST_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CORE_RST_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CORE_RST_IN_Pin */
  GPIO_InitStruct.Pin = CORE_RST_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CORE_RST_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CORE_3V3_5V_PG_Pin CORE_PWR_ISO_PG_Pin V3_PG_Pin V3STB_PG_Pin */
  GPIO_InitStruct.Pin = CORE_3V3_5V_PG_Pin|CORE_PWR_ISO_PG_Pin|V3_PG_Pin|V3STB_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CORE_PWR_CONN_PG_Pin */
  GPIO_InitStruct.Pin = CORE_PWR_CONN_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CORE_PWR_CONN_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins for I2C2 : I2C2_SCL_Pin I2C2_SDA_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure GPIO pins for analog input
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1; // Assuming ADC_CHANNEL_8 is on PA0 and ADC_CHANNEL_9 is on PA1
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);




/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
