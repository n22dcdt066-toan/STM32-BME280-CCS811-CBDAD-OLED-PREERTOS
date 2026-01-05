/**
  ******************************************************************************
  * @file    BH1750.c
  * @brief   BH1750 Light Sensor driver implementation for STM32F411 with HAL I2C2
  * @author  Generated for STM32F411CEU6 RTOS Project
  * @date    2025
  ******************************************************************************
  * @attention
  *
  * This driver provides comprehensive support for BH1750FVI Digital Light Sensor:
  * - I2C Communication on I2C2 interface
  * - Continuous and One-time measurement modes
  * - High resolution (0.5 lux) and Low resolution (4 lux) modes
  * - Automatic power management
  * - Non-blocking and blocking read operations
  *
  ******************************************************************************
  */

#include "BH1750_STM32.h"
#include "main.h"

// Private variables
static uint8_t bh1750_address = BH1750_DEFAULT_ADDRESS;
static BH1750_Mode_t current_mode = BH1750_CONTINUOUS_HIGH_RES_MODE;
static bool device_initialized = false;

/**
  * @brief  Initialize BH1750 sensor
  * @param  address: I2C address (BH1750_ADDRESS_LOW or BH1750_ADDRESS_HIGH)
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_Init(uint8_t address)
{
    BH1750_Status_t status;
    
    // Set device address
    bh1750_address = (address == BH1750_ADDRESS_HIGH) ? BH1750_ADDRESS_HIGH : BH1750_ADDRESS_LOW;
    
    // Reset the sensor
    status = BH1750_Reset();
    if (status != BH1750_OK) {
        return status;
    }
    
    // Wait for reset to complete
    HAL_Delay(10);
    
    // Power on the sensor
    status = BH1750_PowerOn();
    if (status != BH1750_OK) {
        return status;
    }
    
    // Set default measurement mode (High Resolution Mode 2 - 0.5 lux resolution)
    status = BH1750_SetMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
    if (status != BH1750_OK) {
        return status;
    }
    
    // Wait for first measurement
    HAL_Delay(180);  // 120ms measurement time + margin
    
    device_initialized = true;
    return BH1750_OK;
}

/**
  * @brief  Reset BH1750 sensor
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_Reset(void)
{
    uint8_t command = BH1750_RESET;
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, 
                                        (bh1750_address << 1), 
                                        &command, 1, 
                                        BH1750_I2C_TIMEOUT);
    
    return (hal_status == HAL_OK) ? BH1750_OK : BH1750_ERROR;
}

/**
  * @brief  Power on BH1750 sensor
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_PowerOn(void)
{
    uint8_t command = BH1750_POWER_ON;
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, 
                                        (bh1750_address << 1), 
                                        &command, 1, 
                                        BH1750_I2C_TIMEOUT);
    
    return (hal_status == HAL_OK) ? BH1750_OK : BH1750_ERROR;
}

/**
  * @brief  Power down BH1750 sensor
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_PowerDown(void)
{
    uint8_t command = BH1750_POWER_DOWN;
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, 
                                        (bh1750_address << 1), 
                                        &command, 1, 
                                        BH1750_I2C_TIMEOUT);
    
    device_initialized = false;
    return (hal_status == HAL_OK) ? BH1750_OK : BH1750_ERROR;
}

/**
  * @brief  Set BH1750 measurement mode
  * @param  mode: Measurement mode
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_SetMode(BH1750_Mode_t mode)
{
    uint8_t command = (uint8_t)mode;
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, 
                                        (bh1750_address << 1), 
                                        &command, 1, 
                                        BH1750_I2C_TIMEOUT);
    
    if (hal_status == HAL_OK) {
        current_mode = mode;
        return BH1750_OK;
    }
    
    return BH1750_ERROR;
}

/**
  * @brief  Set BH1750 measurement time register (sensitivity adjustment)
  * @param  mtreg: Measurement time register value (31-254, default 69)
  * @retval BH1750_Status_t
  * @note   This allows sensitivity adjustment from 0.45x to 3.68x
  */
BH1750_Status_t BH1750_SetMTreg(uint8_t mtreg)
{
    HAL_StatusTypeDef hal_status;
    
    // Validate input range
    if (mtreg < 31 || mtreg > 254) {
        return BH1750_ERROR;
    }
    
    // Change Measurement time High bit
    uint8_t high_bit = (0x40 | (mtreg >> 5));
    hal_status = HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, 
                                        (bh1750_address << 1), 
                                        &high_bit, 1, 
                                        BH1750_I2C_TIMEOUT);
    if (hal_status != HAL_OK) return BH1750_ERROR;
    
    // Change Measurement time Low bit
    uint8_t low_bit = (0x60 | (mtreg & 0x1F));
    hal_status = HAL_I2C_Master_Transmit(&BH1750_I2C_HANDLE, 
                                        (bh1750_address << 1), 
                                        &low_bit, 1, 
                                        BH1750_I2C_TIMEOUT);
    
    return (hal_status == HAL_OK) ? BH1750_OK : BH1750_ERROR;
}

/**
  * @brief  Read light intensity from BH1750
  * @param  lux: Pointer to store light intensity value
  * @retval BH1750_Status_t
  * @note   This function reads the current measurement without triggering new one
  */
BH1750_Status_t BH1750_ReadLight(float* lux)
{
    if (!device_initialized || lux == NULL) {
        return BH1750_ERROR;
    }
    
    uint8_t data[2];
    HAL_StatusTypeDef hal_status;
    
    // Read 2 bytes from sensor
    hal_status = HAL_I2C_Master_Receive(&BH1750_I2C_HANDLE, 
                                       (bh1750_address << 1) | 0x01, 
                                       data, 2, 
                                       BH1750_I2C_TIMEOUT);
    
    if (hal_status != HAL_OK) {
        *lux = 0.0f;
        return BH1750_ERROR;
    }
    
    // Combine the two bytes
    uint16_t raw_value = (data[0] << 8) | data[1];
    
    // Convert to lux based on mode
    switch (current_mode) {
        case BH1750_CONTINUOUS_HIGH_RES_MODE:
        case BH1750_ONETIME_HIGH_RES_MODE:
            *lux = raw_value / 1.2f;  // 1 lux resolution
            break;
            
        case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
        case BH1750_ONETIME_HIGH_RES_MODE_2:
            *lux = raw_value / 2.4f;  // 0.5 lux resolution  
            break;
            
        case BH1750_CONTINUOUS_LOW_RES_MODE:
        case BH1750_ONETIME_LOW_RES_MODE:
            *lux = raw_value / 1.2f;  // 4 lux resolution (same formula, but inherently less precise)
            break;
            
        default:
            *lux = raw_value / 2.4f;  // Default to high resolution mode 2
            break;
    }
    
    return BH1750_OK;
}

/**
  * @brief  Trigger measurement and read light intensity (blocking)
  * @param  lux: Pointer to store light intensity value
  * @param  mode: Measurement mode to use
  * @retval BH1750_Status_t
  * @note   This function is useful for one-time measurements
  */
BH1750_Status_t BH1750_ReadLightBlocking(float* lux, BH1750_Mode_t mode)
{
    BH1750_Status_t status;
    
    if (lux == NULL) {
        return BH1750_ERROR;
    }
    
    // Set measurement mode
    status = BH1750_SetMode(mode);
    if (status != BH1750_OK) {
        return status;
    }
    
    // Wait for measurement to complete
    uint16_t delay_ms;
    switch (mode) {
        case BH1750_ONETIME_LOW_RES_MODE:
        case BH1750_CONTINUOUS_LOW_RES_MODE:
            delay_ms = 24;  // 16ms + margin
            break;
        default:
            delay_ms = 180; // 120ms + margin
            break;
    }
    
    HAL_Delay(delay_ms);
    
    // Read the result
    return BH1750_ReadLight(lux);
}

/**
  * @brief  Trigger measurement in one-time mode
  * @param  mode: One-time measurement mode
  * @retval BH1750_Status_t
  * @note   Use BH1750_ReadLight() after appropriate delay to get result
  */
BH1750_Status_t BH1750_TriggerMeasurement(BH1750_Mode_t mode)
{
    // Only allow one-time modes for this function
    if (mode != BH1750_ONETIME_HIGH_RES_MODE &&
        mode != BH1750_ONETIME_HIGH_RES_MODE_2 &&
        mode != BH1750_ONETIME_LOW_RES_MODE) {
        return BH1750_ERROR;
    }
    
    return BH1750_SetMode(mode);
}

/**
  * @brief  Check if BH1750 is connected and responding
  * @retval true if connected, false otherwise
  */
bool BH1750_IsConnected(void)
{
    HAL_StatusTypeDef hal_status;
    
    // Try to communicate with device
    hal_status = HAL_I2C_IsDeviceReady(&BH1750_I2C_HANDLE, 
                                      (bh1750_address << 1), 
                                      3, 
                                      BH1750_I2C_TIMEOUT);
    
    return (hal_status == HAL_OK);
}

/**
  * @brief  Read all BH1750 data and status
  * @param  data: Pointer to BH1750_Data_t structure
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_ReadAll(BH1750_Data_t* data)
{
    if (data == NULL) {
        return BH1750_ERROR;
    }
    
    BH1750_Status_t status = BH1750_ReadLight(&data->lux);
    
    data->mode = current_mode;
    data->device_address = bh1750_address;
    data->connected = BH1750_IsConnected();
    data->valid = (status == BH1750_OK);
    data->last_read_ms = HAL_GetTick();
    
    return status;
}

/**
  * @brief  Get BH1750 status information
  * @param  status: Pointer to BH1750_Data_t structure for status
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_GetStatus(BH1750_Data_t* status)
{
    if (status == NULL) {
        return BH1750_ERROR;
    }
    
    status->mode = current_mode;
    status->device_address = bh1750_address;
    status->connected = BH1750_IsConnected();
    status->valid = device_initialized;
    status->last_read_ms = HAL_GetTick();
    status->lux = 0.0f;  // No actual reading
    
    return BH1750_OK;
}