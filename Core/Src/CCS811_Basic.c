/**
  ******************************************************************************
  * @file    CCS811_Basic.c
  * @brief   CCS811 air quality sensor driver for STM32F411 with HAL I2C
  * @author  Generated for STM32F411CEU6 RTOS Project
  ******************************************************************************
  */

#include "CCS811.h"

// Private variables
static uint8_t ccs811_i2c_address = 0;
static bool ccs811_app_started = false;

// Private function prototypes
static HAL_StatusTypeDef CCS811_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef CCS811_ReadRegister(uint8_t reg, uint8_t* value);
static HAL_StatusTypeDef CCS811_ReadRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
static HAL_StatusTypeDef CCS811_WriteCommand(uint8_t command);

/**
  * @brief  Initialize CCS811 sensor
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t hw_id, hw_version;
    uint8_t status_reg;
    
    // Try primary address first
    ccs811_i2c_address = CCS811_I2C_ADDR_PRIM;
    status = CCS811_ReadRegister(CCS811_HW_ID, &hw_id);
    
    if (status != HAL_OK || hw_id != CCS811_HW_ID_CODE) {
        // Try secondary address
        ccs811_i2c_address = CCS811_I2C_ADDR_SEC;
        status = CCS811_ReadRegister(CCS811_HW_ID, &hw_id);
        
        if (status != HAL_OK || hw_id != CCS811_HW_ID_CODE) {
            return HAL_ERROR;
        }
    }
    
    // Read hardware version
    status = CCS811_ReadRegister(CCS811_HW_VERSION, &hw_version);
    if (status != HAL_OK) return status;
    
    // Check if application is valid
    status = CCS811_ReadRegister(CCS811_STATUS, &status_reg);
    if (status != HAL_OK) return status;
    
    if (!(status_reg & CCS811_STATUS_APP_VALID)) {
        return HAL_ERROR; // Application not valid
    }
    
    // Start application if not already started
    if (!(status_reg & CCS811_STATUS_FW_MODE)) {
        status = CCS811_StartApp();
        if (status != HAL_OK) return status;
        
        HAL_Delay(10); // Wait for app to start
        
        // Check again
        status = CCS811_ReadRegister(CCS811_STATUS, &status_reg);
        if (status != HAL_OK) return status;
        
        if (!(status_reg & CCS811_STATUS_FW_MODE)) {
            return HAL_ERROR; // Failed to start app
        }
    }
    
    ccs811_app_started = true;
    
    // Set drive mode to 1 second intervals
    status = CCS811_SetDriveMode(CCS811_DRIVE_MODE_1SEC);
    if (status != HAL_OK) return status;
    
    return HAL_OK;
}

/**
  * @brief  Check if CCS811 is connected
  * @retval true if connected, false otherwise
  */
bool CCS811_IsConnected(void)
{
    uint8_t hw_id;
    HAL_StatusTypeDef status;
    
    if (ccs811_i2c_address == 0) return false;
    
    status = CCS811_ReadRegister(CCS811_HW_ID, &hw_id);
    return (status == HAL_OK && hw_id == CCS811_HW_ID_CODE);
}

/**
  * @brief  Start CCS811 application
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_StartApp(void)
{
    return CCS811_WriteCommand(CCS811_BOOTLOADER_APP_START);
}

/**
  * @brief  Soft reset CCS811
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_SoftReset(void)
{
    uint8_t reset_sequence[] = {0x11, 0xE5, 0x72, 0x8A};
    HAL_StatusTypeDef status;
    
    // Write reset sequence to SW_RESET register
    status = HAL_I2C_Mem_Write(&CCS811_I2C_HANDLE, ccs811_i2c_address, 
                               CCS811_SW_RESET, 1, reset_sequence, 4, CCS811_I2C_TIMEOUT);
    
    if (status == HAL_OK) {
        HAL_Delay(10);
        ccs811_app_started = false;
    }
    
    return status;
}

/**
  * @brief  Set CCS811 drive mode
  * @param  mode: Drive mode (see CCS811_DRIVE_MODE_* defines)
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_SetDriveMode(uint8_t mode)
{
    if (!ccs811_app_started) return HAL_ERROR;
    
    return CCS811_WriteRegister(CCS811_MEAS_MODE, mode);
}

/**
  * @brief  Set environmental data for compensation
  * @param  humidity: Relative humidity (0-100%)
  * @param  temperature: Temperature in Celsius
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_SetEnvironmentalData(float humidity, float temperature)
{
    if (!ccs811_app_started) return HAL_ERROR;
    
    // Convert humidity to CCS811 format
    // Humidity: ((humidity % + 0.5) / 100) * 65535
    uint16_t hum_data = (uint16_t)((humidity + 0.5f) / 100.0f * 65535.0f);
    
    // Convert temperature to CCS811 format  
    // Temperature: (temperature + 25) * 512
    uint16_t temp_data = (uint16_t)((temperature + 25.0f) * 512.0f);
    
    uint8_t env_data[4];
    env_data[0] = (hum_data >> 8) & 0xFF;   // Humidity MSB
    env_data[1] = hum_data & 0xFF;          // Humidity LSB  
    env_data[2] = (temp_data >> 8) & 0xFF;  // Temperature MSB
    env_data[3] = temp_data & 0xFF;         // Temperature LSB
    
    return HAL_I2C_Mem_Write(&CCS811_I2C_HANDLE, ccs811_i2c_address,
                             CCS811_ENV_DATA, 1, env_data, 4, CCS811_I2C_TIMEOUT);
}

/**
  * @brief  Check if data is ready
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_CheckDataReady(void)
{
    uint8_t status;
    HAL_StatusTypeDef hal_status;
    
    hal_status = CCS811_ReadRegister(CCS811_STATUS, &status);
    if (hal_status != HAL_OK) return hal_status;
    
    return (status & CCS811_STATUS_DATA_READY) ? HAL_OK : HAL_ERROR;
}

/**
  * @brief  Read sensor data
  * @param  data: Pointer to data structure
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_ReadData(CCS811_Data_t* data)
{
    HAL_StatusTypeDef status;
    uint8_t raw_data[8];
    
    data->valid = false;
    
    if (!ccs811_app_started) return HAL_ERROR;
    
    // Check if data is ready
    status = CCS811_CheckDataReady();
    if (status != HAL_OK) return status;
    
    // Read algorithm result data (8 bytes)
    status = CCS811_ReadRegisters(CCS811_ALG_RESULT_DATA, raw_data, 8);
    if (status != HAL_OK) return status;
    
    // Parse data
    data->co2 = (raw_data[0] << 8) | raw_data[1];
    data->tvoc = (raw_data[2] << 8) | raw_data[3];
    data->status = raw_data[4];
    data->error_id = raw_data[5];
    data->raw_data = (raw_data[6] << 8) | raw_data[7];
    
    // Extract current and raw voltage from raw_data
    data->current = (data->raw_data >> 10) & 0x3F;
    data->raw_voltage = data->raw_data & 0x3FF;
    
    // Check for errors
    if (data->status & CCS811_STATUS_ERROR) {
        return HAL_ERROR;
    }
    
    data->valid = true;
    return HAL_OK;
}

/**
  * @brief  Check for errors
  * @retval Error ID (0 if no error)
  */
uint8_t CCS811_CheckForError(void)
{
    uint8_t status, error_id = 0;
    
    if (CCS811_ReadRegister(CCS811_STATUS, &status) == HAL_OK) {
        if (status & CCS811_STATUS_ERROR) {
            CCS811_ReadRegister(CCS811_ERROR_ID, &error_id);
        }
    }
    
    return error_id;
}

// Private functions implementation

/**
  * @brief  Write command to CCS811
  * @param  command: Command to write
  * @retval HAL status
  */
static HAL_StatusTypeDef CCS811_WriteCommand(uint8_t command)
{
    return HAL_I2C_Master_Transmit(&CCS811_I2C_HANDLE, ccs811_i2c_address, 
                                   &command, 1, CCS811_I2C_TIMEOUT);
}

/**
  * @brief  Write data to CCS811 register
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval HAL status
  */
static HAL_StatusTypeDef CCS811_WriteRegister(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&CCS811_I2C_HANDLE, ccs811_i2c_address, 
                             reg, 1, &value, 1, CCS811_I2C_TIMEOUT);
}

/**
  * @brief  Read data from CCS811 register
  * @param  reg: Register address
  * @param  value: Pointer to store read value
  * @retval HAL status
  */
static HAL_StatusTypeDef CCS811_ReadRegister(uint8_t reg, uint8_t* value)
{
    return HAL_I2C_Mem_Read(&CCS811_I2C_HANDLE, ccs811_i2c_address, 
                            reg, 1, value, 1, CCS811_I2C_TIMEOUT);
}

/**
  * @brief  Read multiple registers from CCS811
  * @param  reg: Starting register address
  * @param  buffer: Buffer to store read data
  * @param  length: Number of bytes to read
  * @retval HAL status
  */
static HAL_StatusTypeDef CCS811_ReadRegisters(uint8_t reg, uint8_t* buffer, uint8_t length)
{
    return HAL_I2C_Mem_Read(&CCS811_I2C_HANDLE, ccs811_i2c_address, 
                            reg, 1, buffer, length, CCS811_I2C_TIMEOUT);
}


