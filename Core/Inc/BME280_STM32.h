#ifndef BME280_STM32_H_
#define BME280_STM32_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// BME280 SPI3 Configuration
extern SPI_HandleTypeDef hspi3;
#define BME280_SPI_HANDLE       hspi3

// BME280 SPI3 Pin Definitions  
#define BME280_CS_GPIO_Port     GPIOB
#define BME280_CS_Pin           GPIO_PIN_13

// SPI Protocol Definitions for BME280
#define BME280_SPI_READ         0x80
#define BME280_SPI_WRITE        0x7F
#define BME280_SPI_TIMEOUT      1000

// BME280 Register Addresses
#define BME280_REG_HUM_LSB      0xFE
#define BME280_REG_HUM_MSB      0xFD
#define BME280_REG_TEMP_XLSB    0xFC
#define BME280_REG_TEMP_LSB     0xFB
#define BME280_REG_TEMP_MSB     0xFA
#define BME280_REG_PRESS_XLSB   0xF9
#define BME280_REG_PRESS_LSB    0xF8
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_RESET        0xE0
#define BME280_REG_ID           0xD0

// BME280 Calibration Registers
#define BME280_REG_CALIB_00     0x88
#define BME280_REG_CALIB_25     0xA1  // dig_H1
#define BME280_REG_CALIB_26     0xE1
#define BME280_REG_CALIB_27     0xE2

// BME280 Chip ID
#define BME280_REG_CHIP_ID      0xD0
#define BME280_CHIP_ID_VALUE    0x60
#define BME280_CHIP_ID          0x60

// BME280 Data Structure
typedef struct {
    float temperature;    // Temperature in Celsius
    float humidity;       // Humidity in %RH
    float pressure;       // Pressure in hPa
    uint8_t chip_id;
    uint8_t status;
    bool valid;           // Data validity flag
} BME280_Data_t;

// BME280 Calibration Data Structure
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BME280_Calibration_t;

// Function Prototypes - SPI3 Version
HAL_StatusTypeDef BME280_Init(void);
HAL_StatusTypeDef BME280_ReadAll(BME280_Data_t* data);
HAL_StatusTypeDef BME280_ReadTemperature(float* temperature);
HAL_StatusTypeDef BME280_ReadHumidity(float* humidity);  
HAL_StatusTypeDef BME280_ReadPressure(float* pressure);
bool BME280_IsConnected(void);
HAL_StatusTypeDef BME280_SoftReset(void);

// SPI3 Low-level Functions
HAL_StatusTypeDef BME280_SPI_WriteRegister(uint8_t reg_addr, uint8_t* data, uint8_t len);
HAL_StatusTypeDef BME280_SPI_ReadRegister(uint8_t reg_addr, uint8_t* data, uint8_t len);

#endif /* BME280_STM32_H_ */
