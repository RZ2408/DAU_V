#ifndef I2C_MAX11611_H
#define I2C_MAX11611_H

#include "stm32f0xx_hal.h"

// Global declaration of MAX11611 ADC values
extern uint32_t max11611_adc_values[4];

void i2c_readMax11611Adc(uint8_t devAddr);

#endif /* I2C_MAX11611_H */
