/**
  ******************************************************************************
  * @file    BH1750_STM32.h
  * @brief   BH1750 Light Sensor driver for STM32F411 with HAL I2C2
  * @author  Generated for STM32F411CEU6 RTOS Project
  * @date    2025
  ******************************************************************************
  * @attention
  *
  * This driver supports BH1750FVI Digital Light Sensor
  * - I2C Communication (I2C2)
  * - Measurement range: 1-65535 lux
  * - Resolution: 0.5 lux (High Resolution Mode 2)
  * - Power: 0.12mA active, 0.01µA power down
  *
  ******************************************************************************
  */

#ifndef BH1750_STM32_H_
#define BH1750_STM32_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// BH1750 I2C Configuration
extern I2C_HandleTypeDef hi2c2;
#define BH1750_I2C_HANDLE       hi2c2
#define BH1750_I2C_TIMEOUT      1000

// BH1750 I2C Address Options
#define BH1750_ADDRESS_LOW      0x23  // ADDR pin connected to GND
#define BH1750_ADDRESS_HIGH     0x5C  // ADDR pin connected to VCC
#define BH1750_DEFAULT_ADDRESS  BH1750_ADDRESS_LOW

// BH1750 Commands
#define BH1750_POWER_DOWN       0x00  // Power down mode
#define BH1750_POWER_ON         0x01  // Power on mode
#define BH1750_RESET            0x07  // Reset data register value

// BH1750 Measurement Modes
typedef enum {
    BH1750_CONTINUOUS_HIGH_RES_MODE   = 0x10,  // 1 lux resolution, 120ms measurement time
    BH1750_CONTINUOUS_HIGH_RES_MODE_2 = 0x11,  // 0.5 lux resolution, 120ms measurement time  
    BH1750_CONTINUOUS_LOW_RES_MODE    = 0x13,  // 4 lux resolution, 16ms measurement time
    BH1750_ONETIME_HIGH_RES_MODE      = 0x20,  // 1 lux resolution, 120ms, auto power down
    BH1750_ONETIME_HIGH_RES_MODE_2    = 0x21,  // 0.5 lux resolution, 120ms, auto power down
    BH1750_ONETIME_LOW_RES_MODE       = 0x23   // 4 lux resolution, 16ms, auto power down
} BH1750_Mode_t;

// BH1750 Status
typedef enum {
    BH1750_OK = 0,
    BH1750_ERROR,
    BH1750_TIMEOUT,
    BH1750_NOT_CONNECTED
} BH1750_Status_t;

// BH1750 Data Structure
typedef struct {
    float lux;              // Light intensity in lux
    BH1750_Mode_t mode;     // Current measurement mode
    uint8_t device_address; // I2C device address
    bool connected;         // Connection status
    bool valid;             // Data validity flag
    uint32_t last_read_ms;  // Last reading timestamp
} BH1750_Data_t;

// Function Prototypes
BH1750_Status_t BH1750_Init(uint8_t address);
BH1750_Status_t BH1750_Reset(void);
BH1750_Status_t BH1750_PowerOn(void);
BH1750_Status_t BH1750_PowerDown(void);
BH1750_Status_t BH1750_SetMode(BH1750_Mode_t mode);
BH1750_Status_t BH1750_SetMTreg(uint8_t mtreg);
BH1750_Status_t BH1750_ReadLight(float* lux);
BH1750_Status_t BH1750_ReadLightBlocking(float* lux, BH1750_Mode_t mode);
BH1750_Status_t BH1750_TriggerMeasurement(BH1750_Mode_t mode);
bool BH1750_IsConnected(void);

// High level functions
BH1750_Status_t BH1750_ReadAll(BH1750_Data_t* data);
BH1750_Status_t BH1750_GetStatus(BH1750_Data_t* status);

#endif /* BH1750_STM32_H_ */