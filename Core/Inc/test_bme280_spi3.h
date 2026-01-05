/**
  ******************************************************************************
  * @file    test_bme280_spi3.h
  * @brief   Header for BME280 SPI3 test functions
  * @author  STM32 BME280 SPI3 Project
  * @date    2025
  ******************************************************************************
  */

#ifndef TEST_BME280_SPI3_H_
#define TEST_BME280_SPI3_H_

#include "stm32f4xx_hal.h"

// Function Prototypes
HAL_StatusTypeDef Test_BME280_SPI3_Communication(void);
HAL_StatusTypeDef Verify_SPI3_Timing(void);
HAL_StatusTypeDef Optimize_SPI3_Performance(void);

#endif /* TEST_BME280_SPI3_H_ */