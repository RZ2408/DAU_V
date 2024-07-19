/*
 * i2c_max11611.c
 *
 *  Created on: Jul 5, 2024
 *      Author: QA
 */

#include "i2c_max11611.h"
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>

// Include UART header

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;

void i2c_readMax11611Adc(uint8_t devAddr) {
    HAL_StatusTypeDef status;
    uint8_t setup, config;
    uint8_t response[2];
    uint16_t val;
    char buffer[50];
    uint32_t volt[12];

    // Setup: AIN/REF=internal ref & ref. output, internal clock, unipolar, no config reset
    setup = 0b11110010;
    status = HAL_I2C_Master_Transmit(&hi2c2, (devAddr << 1), &setup, sizeof(setup), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        char error_msg[] = "Failed to setup ADC MAX11611.\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        return;
    }

    // Read channels
    for (int i = 0; i < 4; i++) {
        // Configure which channels to read
        config = 0b01100001 | (i << 1); // sel[0:2] = read channel as in CS[0:3], single ended
        status = HAL_I2C_Master_Transmit(&hi2c2, (devAddr << 1), &config, sizeof(config), HAL_MAX_DELAY);
        if (status != HAL_OK) {
            char error_msg[50];
            snprintf(error_msg, sizeof(error_msg), "I2C Tx Error on ch %d of MAX11611.\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
            return;
        }

        status = HAL_I2C_Master_Receive(&hi2c2, (devAddr << 1), response, sizeof(response), HAL_MAX_DELAY);
        if (status != HAL_OK) {
            char error_msg[50];
            snprintf(error_msg, sizeof(error_msg), "I2C Rx Error on ch %d of MAX11611.\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
            return;
        }

        val = ((response[0] & 0x03) << 8) | response[1];
        volt[i] = val * 2;  // Adjust this calculation based on your voltage conversion needs

        // Print raw ADC values over UART
        snprintf(buffer, sizeof(buffer), "AIN%d Raw ADC Value: %lu\r\n", i, volt[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}
