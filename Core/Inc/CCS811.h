#ifndef CCS811_H_
#define CCS811_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// CCS811 I2C Configuration
#define CCS811_I2C_HANDLE hi2c2
#define CCS811_I2C_TIMEOUT 1000

// CCS811 I2C Addresses (7-bit, shifted for HAL)
#define CCS811_I2C_ADDR_PRIM    (0x5A << 1)
#define CCS811_I2C_ADDR_SEC     (0x5B << 1)

// CCS811 Register Definitions
#define CCS811_STATUS           0x00
#define CCS811_MEAS_MODE        0x01
#define CCS811_ALG_RESULT_DATA  0x02
#define CCS811_RAW_DATA         0x03
#define CCS811_ENV_DATA         0x05
#define CCS811_NTC              0x06
#define CCS811_THRESHOLDS       0x10
#define CCS811_BASELINE         0x11
#define CCS811_HW_ID            0x20
#define CCS811_HW_VERSION       0x21
#define CCS811_FW_BOOT_VERSION  0x23
#define CCS811_FW_APP_VERSION   0x24
#define CCS811_ERROR_ID         0xE0
#define CCS811_SW_RESET         0xFF

// CCS811 Commands
#define CCS811_BOOTLOADER_APP_ERASE     0xF1
#define CCS811_BOOTLOADER_APP_DATA      0xF2
#define CCS811_BOOTLOADER_APP_VERIFY    0xF3
#define CCS811_BOOTLOADER_APP_START     0xF4

// CCS811 Status bits
#define CCS811_STATUS_FW_MODE           0x80
#define CCS811_STATUS_APP_VALID         0x10
#define CCS811_STATUS_DATA_READY        0x08
#define CCS811_STATUS_ERROR             0x01

// CCS811 Measurement modes
#define CCS811_DRIVE_MODE_IDLE          0x00
#define CCS811_DRIVE_MODE_1SEC          0x10
#define CCS811_DRIVE_MODE_10SEC         0x20
#define CCS811_DRIVE_MODE_60SEC         0x30
#define CCS811_DRIVE_MODE_250MS         0x40

// Hardware ID expected value
#define CCS811_HW_ID_CODE               0x81

// CCS811 Data Structure
typedef struct {
    uint16_t co2;        // eCO2 in ppm
    uint16_t tvoc;       // TVOC in ppb
    uint8_t status;
    uint8_t error_id;
    uint16_t raw_data;
    uint8_t current;
    uint16_t raw_voltage;
    bool valid;          // Data validity flag
} CCS811_Data_t;

// Function Prototypes
HAL_StatusTypeDef CCS811_Init(void);
HAL_StatusTypeDef CCS811_ReadData(CCS811_Data_t* data);
HAL_StatusTypeDef CCS811_SetDriveMode(uint8_t mode);
HAL_StatusTypeDef CCS811_SetEnvironmentalData(float humidity, float temperature);
HAL_StatusTypeDef CCS811_CheckDataReady(void);
HAL_StatusTypeDef CCS811_SoftReset(void);
bool CCS811_IsConnected(void);
uint8_t CCS811_CheckForError(void);
HAL_StatusTypeDef CCS811_StartApp(void);

// External I2C handle declaration
extern I2C_HandleTypeDef hi2c2;

#endif /* CCS811_H_ */
