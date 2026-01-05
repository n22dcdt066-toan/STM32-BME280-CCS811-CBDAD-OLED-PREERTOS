/**
  ******************************************************************************
  * @file    BME280.c
  * @brief   BME280 sensor driver for STM32F411 with HAL SPI3
  * @author  Generated for STM32F411CEU6 RTOS Project
  ******************************************************************************
  */

#include "BME280_STM32.h"
#include "main.h"

// Private variables
static BME280_Calibration_t cal_data;
static bool calibration_loaded = false;
static int32_t t_fine; // Temperature fine value for pressure and humidity calculation

// Private function prototypes
static HAL_StatusTypeDef BME280_ReadCalibrationData(void);
static int32_t BME280_CompensateTemperature(int32_t adc_T);
static uint32_t BME280_CompensatePressure(int32_t adc_P);
static uint32_t BME280_CompensateHumidity(int32_t adc_H);

/**
  * @brief  Initialize BME280 sensor
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;
    
    // Read chip ID via SPI3
    status = BME280_SPI_ReadRegister(BME280_REG_CHIP_ID, &chip_id, 1);
    if (status != HAL_OK || chip_id != BME280_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }
    
    // Soft reset
    uint8_t reset_cmd = 0xB6;
    status = BME280_SPI_WriteRegister(BME280_REG_RESET, &reset_cmd, 1);
    if (status != HAL_OK) return status;
    
    HAL_Delay(10);
    
    // Read calibration data
    status = BME280_ReadCalibrationData();
    if (status != HAL_OK) return status;
    
    // Set humidity oversampling (x1) 
    uint8_t hum_ctrl = 0x01;
    status = BME280_SPI_WriteRegister(BME280_REG_CTRL_HUM, &hum_ctrl, 1); 
    if (status != HAL_OK) return status;
    
    // Set temperature and pressure oversampling, and forced mode (Temp x1, Press x1, Forced mode)
    uint8_t meas_ctrl = 0x25; // 001 001 01 - temp x1, press x1, forced mode
    status = BME280_SPI_WriteRegister(BME280_REG_CTRL_MEAS, &meas_ctrl, 1);
    if (status != HAL_OK) return status;
    
    // Set config register (1000ms standby, filter off)
    uint8_t config_reg = 0xA0;
    status = BME280_SPI_WriteRegister(BME280_REG_CONFIG, &config_reg, 1);
    if (status != HAL_OK) return status;
    
    // Wait for first measurement
    HAL_Delay(100);
    if (status != HAL_OK) return status;
    
    calibration_loaded = true;
    return HAL_OK;
}

/**
  * @brief  Check if BME280 is connected
  * @retval true if connected, false otherwise
  */
bool BME280_IsConnected(void)
{
    uint8_t chip_id = 0;
    HAL_StatusTypeDef status = BME280_SPI_ReadRegister(BME280_REG_CHIP_ID, &chip_id, 1);
    
    return (status == HAL_OK && chip_id == BME280_CHIP_ID_VALUE);
}

/**
  * @brief  Soft reset BME280
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_SoftReset(void)
{
    uint8_t reset_cmd = 0xB6;
    HAL_StatusTypeDef status = BME280_SPI_WriteRegister(BME280_REG_RESET, &reset_cmd, 1);
    
    if (status == HAL_OK) {
        HAL_Delay(10);
        calibration_loaded = false;  // Need to reload calibration after reset
    }
    
    return status;
}

/**
  * @brief  Read all sensor data (temperature, humidity, pressure)
  * @param  data: Pointer to data structure
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_ReadAll(BME280_Data_t* data)
{
    HAL_StatusTypeDef status;
    uint8_t raw_data[8];
    int32_t adc_T, adc_P, adc_H;
    
    data->valid = false;
    
    if (!calibration_loaded) {
        return HAL_ERROR;
    }
    
    // Trigger forced mode measurement
    uint8_t meas_ctrl = 0x25; // temp x1, press x1, forced mode
    status = BME280_SPI_WriteRegister(BME280_REG_CTRL_MEAS, &meas_ctrl, 1);
    if (status != HAL_OK) return status;
    
    // Wait for measurement to complete
    HAL_Delay(10);
    
    // Read all data registers at once (0xF7 to 0xFE)
    status = BME280_SPI_ReadRegister(BME280_REG_PRESS_MSB, raw_data, 8);
    if (status != HAL_OK) return status;
    
    // Parse raw data
    adc_P = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
    adc_T = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
    adc_H = (raw_data[6] << 8) | raw_data[7];
    
    // Skip if data is invalid (all bits set)
    if (adc_T == 0x80000 || adc_P == 0x80000 || adc_H == 0x8000) {
        return HAL_ERROR;
    }
    
    // Compensate temperature first (needed for pressure and humidity)
    int32_t temp_int = BME280_CompensateTemperature(adc_T);
    data->temperature = temp_int / 100.0f;
    
    // Compensate pressure
    uint32_t press_int = BME280_CompensatePressure(adc_P);
    data->pressure = press_int / 256.0f / 100.0f; // Convert to hPa
    
    // Compensate humidity
    uint32_t hum_int = BME280_CompensateHumidity(adc_H);
    data->humidity = hum_int / 1024.0f;
    
    // Read status
    status = BME280_SPI_ReadRegister(BME280_REG_STATUS, &data->status, 1);
    if (status != HAL_OK) return status;
    
    data->chip_id = BME280_CHIP_ID;
    data->valid = true;
    
    return HAL_OK;
}

/**
  * @brief  Read temperature only
  * @param  temperature: Pointer to temperature value
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_ReadTemperature(float* temperature)
{
    BME280_Data_t data;
    HAL_StatusTypeDef status = BME280_ReadAll(&data);
    if (status == HAL_OK && data.valid) {
        *temperature = data.temperature;
    }
    return status;
}

/**
  * @brief  Read humidity only
  * @param  humidity: Pointer to humidity value
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_ReadHumidity(float* humidity)
{
    BME280_Data_t data;
    HAL_StatusTypeDef status = BME280_ReadAll(&data);
    if (status == HAL_OK && data.valid) {
        *humidity = data.humidity;
    }
    return status;
}

/**
  * @brief  Read pressure only
  * @param  pressure: Pointer to pressure value
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_ReadPressure(float* pressure)
{
    BME280_Data_t data;
    HAL_StatusTypeDef status = BME280_ReadAll(&data);
    if (status == HAL_OK && data.valid) {
        *pressure = data.pressure;
    }
    return status;
}

// Private functions implementation

/**
  * @brief  Read calibration data from BME280
  * @retval HAL status
  */
static HAL_StatusTypeDef BME280_ReadCalibrationData(void)
{
    HAL_StatusTypeDef status;
    uint8_t calib_data[24];
    uint8_t calib_data_h[7];
    uint8_t dig_H1_data;
    
    // Read calibration data for temperature and pressure (0x88 to 0x9F) - 24 bytes
    status = BME280_SPI_ReadRegister(BME280_REG_CALIB_00, calib_data, 24);
    if (status != HAL_OK) return status;
    
    // Read dig_H1 from 0xA1
    status = BME280_SPI_ReadRegister(BME280_REG_CALIB_25, &dig_H1_data, 1);
    if (status != HAL_OK) return status;
    
    // Read calibration data for humidity (0xE1 to 0xE7) - 7 bytes  
    status = BME280_SPI_ReadRegister(BME280_REG_CALIB_26, calib_data_h, 7);
    if (status != HAL_OK) return status;
    
    // Parse temperature calibration data
    cal_data.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    cal_data.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    cal_data.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    
    // Parse pressure calibration data
    cal_data.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    cal_data.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    cal_data.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    cal_data.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    cal_data.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    cal_data.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    cal_data.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    cal_data.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    cal_data.dig_P9 = (calib_data[23] << 8) | calib_data[22];
    
    // Parse humidity calibration data
    cal_data.dig_H1 = dig_H1_data; // From 0xA1
    cal_data.dig_H2 = (calib_data_h[1] << 8) | calib_data_h[0];
    cal_data.dig_H3 = calib_data_h[2];
    cal_data.dig_H4 = (calib_data_h[3] << 4) | (calib_data_h[4] & 0x0F);
    cal_data.dig_H5 = (calib_data_h[5] << 4) | (calib_data_h[4] >> 4);
    cal_data.dig_H6 = calib_data_h[6];
    
    return HAL_OK;
}

/**
  * @brief  Compensate temperature reading
  * @param  adc_T: Raw temperature ADC value
  * @retval Compensated temperature in 0.01°C
  */
static int32_t BME280_CompensateTemperature(int32_t adc_T)
{
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)cal_data.dig_T1 << 1))) * ((int32_t)cal_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)cal_data.dig_T1)) * 
              ((adc_T >> 4) - ((int32_t)cal_data.dig_T1))) >> 12) * 
              ((int32_t)cal_data.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    return T;
}

/**
  * @brief  Compensate pressure reading
  * @param  adc_P: Raw pressure ADC value
  * @retval Compensated pressure in Pa (Q24.8 format)
  */
static uint32_t BME280_CompensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)cal_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal_data.dig_P3) >> 8) + ((var1 * (int64_t)cal_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal_data.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal_data.dig_P7) << 4);
    
    return (uint32_t)p;
}

/**
  * @brief  Compensate humidity reading
  * @param  adc_H: Raw humidity ADC value
  * @retval Compensated humidity in %RH (Q22.10 format)
  */
static uint32_t BME280_CompensateHumidity(int32_t adc_H)
{
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal_data.dig_H4) << 20) - (((int32_t)cal_data.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)cal_data.dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)cal_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)cal_data.dig_H2) + 8192) >> 14));
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)cal_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    
    return (uint32_t)(v_x1_u32r >> 12);
}

/**
  * @brief  Write data to BME280 register via SPI3
  * @param  reg_addr: Register address
  * @param  data: Pointer to data to write
  * @param  len: Number of bytes to write
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_SPI_WriteRegister(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2];
    
    if (len != 1) {
        return HAL_ERROR; // Only single byte writes supported
    }
    
    // Pull CS low
    HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_RESET);
    
    // Prepare command (MSB = 0 for write)
    tx_buffer[0] = reg_addr & 0x7F;
    tx_buffer[1] = *data;
    
    // Transmit register address and data
    status = HAL_SPI_Transmit(&BME280_SPI_HANDLE, tx_buffer, 2, BME280_SPI_TIMEOUT);
    
    // Pull CS high
    HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
    
    return status;
}

/**
  * @brief  Read data from BME280 register via SPI3
  * @param  reg_addr: Register address
  * @param  data: Pointer to buffer for read data
  * @param  len: Number of bytes to read
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_SPI_ReadRegister(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    HAL_StatusTypeDef status;
    
    // For multi-byte reads, read one by one to ensure compatibility
    for (uint8_t i = 0; i < len; i++) {
        uint8_t tx_cmd = (reg_addr + i) | 0x80; // MSB = 1 for read
        
        // Pull CS low
        HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_RESET);
        
        // Send register address
        status = HAL_SPI_Transmit(&BME280_SPI_HANDLE, &tx_cmd, 1, BME280_SPI_TIMEOUT);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
            return status;
        }
        
        // Read single byte
        status = HAL_SPI_Receive(&BME280_SPI_HANDLE, &data[i], 1, BME280_SPI_TIMEOUT);
        
        // Pull CS high
        HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
        
        if (status != HAL_OK) return status;
        
        // Small delay between reads
        HAL_Delay(1);
    }
    
    return HAL_OK;
}


