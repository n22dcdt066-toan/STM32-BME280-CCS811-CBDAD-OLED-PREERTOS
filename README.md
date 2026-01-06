# HỆ THỐNG GIÁM SÁT MÔI TRƯỜNG STM32F411CEU6 + FreeRTOS

**Sinh viên thực hiện:** N22DCDT066 - Toàn  
**Dự án:** Hệ thống đọc dữ liệu cảm biến môi trường sử dụng FreeRTOS trên STM32F411CEU6

---

## MỤC LỤC

1. [Giới thiệu chung về hệ thống](#1-giới-thiệu-chung-về-hệ-thống)
2. [Các giao tiếp ngoại vi](#2-các-giao-tiếp-ngoại-vi)
3. [FreeRTOS - Tasks và cơ chế hoạt động](#3-freertos---tasks-và-cơ-chế-hoạt-động)
4. [FreeRTOS - Queue, Semaphore, Mutex](#4-freertos---queue-semaphore-mutex)
5. [Cấu hình hệ thống](#5-cấu-hình-hệ-thống)
6. [Hướng dẫn sử dụng](#6-hướng-dẫn-sử-dụng)

---

## 1. GIỚI THIỆU CHUNG VỀ HỆ THỐNG

### 1.1. Tổng quan

Hệ thống giám sát môi trường sử dụng STM32F411CEU6 (Black Pill) kết hợp với FreeRTOS để đọc dữ liệu từ nhiều cảm biến, hiển thị trên màn hình OLED và truyền thông tin qua UART đến Orange Pi 4A.

**Mục đích:**
- Giám sát liên tục các thông số môi trường (nhiệt độ, độ ẩm, áp suất, CO2, TVOC, ánh sáng, độ ẩm đất)
- Hiển thị thông tin realtime trên màn hình OLED 0.96"
- Xử lý đa nhiệm (multitasking) hiệu quả với FreeRTOS
- Truyền dữ liệu thời gian thực qua UART
- Báo hiệu trạng thái hệ thống qua LED

### 1.2. Phần cứng

**Vi điều khiển:**
- **STM32F411CEU6** (Black Pill)
  - CPU: ARM Cortex-M4 @ 100 MHz
  - Flash: 512 KB
  - RAM: 128 KB
  - FPU:  Có hỗ trợ tính toán số thực

**Cảm biến và thiết bị:**
- **BME280**:  Cảm biến môi trường (Nhiệt độ, Độ ẩm, Áp suất) - **Giao tiếp SPI3**
- **CCS811**:  Cảm biến chất lượng không khí (eCO2, TVOC) - **Giao tiếp I2C2**
- **BH1750**: Cảm biến cường độ ánh sáng (Lux) - **Giao tiếp I2C2**
- **Soil Moisture Sensor**: Cảm biến độ ẩm đất - **Giao tiếp ADC1**
- **SSD1306 OLED 0.96"**: Màn hình hiển thị 128x64 - **Giao tiếp I2C3**

**Thiết bị ngoại vi:**
- **LED PC13**: Báo trạng thái hệ thống (Active LOW)
- **Orange Pi 4A**: Nhận dữ liệu qua UART2

### 1.3. Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────────────────────┐
│                   STM32F411CEU6 + FreeRTOS                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │              FreeRTOS Scheduler                           │     │
│  │  (Preemptive Priority-based Multitasking)                │     │
│  └───────────────────────────────────────────────────────────┘     │
│                          │                                          │
│         ┌────────────────┼────────────────┬───────────────┐        │
│         │                │                │               │        │
│    ┌────▼─────┐    ┌────▼─────┐    ┌────▼─────┐    ┌────▼─────┐  │
│    │ Sensor   │    │   UART   │    │   LED    │    │   OLED   │  │
│    │   Task   │    │   Task   │    │   Task   │    │   Task   │  │
│    │(Priority │    │(Priority │    │(Priority │    │(Priority │  │
│    │  High)   │    │ AbvNorm) │    │  BelNorm)│    │  Normal) │  │
│    └────┬─────┘    └────▲─────┘    └──────────┘    └────▲─────┘  │
│         │               │                                │         │
│         │  ┌──────────┐ │                                │         │
│         └─►│  Queue   │─┴────────────────────────────────┘         │
│            │(Sensor   │                                            │
│            │ Data)    │                                            │
│            └──────────┘                                            │
│                                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐          │
│  │SPI Mutex │  │I2C Mutex │  │UART Mutex│  │OLED Mutex│          │
│  │(BME280)  │  │(CCS811,  │  │ (UART2)  │  │ (I2C3)   │          │
│  │          │  │ BH1750)  │  │          │  │          │          │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘          │
│                                                                     │
│  SystemMonitor Task (Priority:  Low) - Heartbeat & Monitor          │
└─────────────────────────────────────────────────────────────────────┘
         │              │              │               │
    ┌────▼────┐    ┌────▼────┐   ┌────▼──────┐  ┌────▼────┐
    │  SPI3   │    │  I2C2   │   │  UART2    │  │  I2C3   │
    │  Bus    │    │  Bus    │   │ (115200)  │  │  Bus    │
    └────┬────┘    └────┬────┘   └────┬──────┘  └────┬────┘
         │              │              │               │
    ┌────▼────┐    ┌────▼────┐   ┌────▼──────┐  ┌────▼────┐
    │ BME280  │    │ CCS811  │   │Orange Pi  │  │SSD1306  │
    │ (Temp,  │    │ BH1750  │   │   4A      │  │  OLED   │
    │Humi,    │    │ (CO2,   │   └───────────┘  │ 128x64  │
    │Press)   │    │TVOC,Lux)│                  └─────────┘
    └─────────┘    └─────────┘
         
    ┌────────────┐
    │    ADC1    │
    │  PA0/PA1   │
    └─────┬──────┘
          │
    ┌─────▼──────┐
    │Soil Moisture│
    │   Sensor    │
    └────────────┘
```

### 1.4. Nguyên lý hoạt động

1. **SensorReadingTask** đọc dữ liệu từ: 
   - BME280 qua SPI3 (với SPI mutex protection)
   - CCS811 và BH1750 qua I2C2 (với I2C mutex protection)
   - Soil Moisture qua ADC1
2.  Dữ liệu được đưa vào **Queue** (sensorDataQueue)
3. **UartCommunicationTask** lấy dữ liệu từ Queue, format thành JSON/CSV/Plain text
4. Dữ liệu được gửi qua UART2 đến Orange Pi 4A (với UART mutex protection)
5. **OledDisplayTask** hiển thị thông tin realtime trên màn hình OLED (với OLED mutex protection)
6. **LedStatusTask** cập nhật trạng thái LED dựa trên tình trạng cảm biến
7. **SystemMonitorTask** gửi heartbeat message định kỳ và giám sát hệ thống

---

## 2. CÁC GIAO TIẾP NGOẠI VI

### 2.1. SPI3 (Serial Peripheral Interface)

**Mục đích:** Giao tiếp với cảm biến BME280

#### Cấu hình phần cứng
```c
// Pin Configuration
PA15 (SPI3_NSS)  :  Chip Select (CS) cho BME280 (GPIO manual control)
PB3  (SPI3_SCK)  : Serial Clock
PB4  (SPI3_MISO) : Master In Slave Out
PB5  (SPI3_MOSI) : Master Out Slave In

// SPI Settings
Mode          : Master Mode
Direction     : Full-Duplex
Data Size     : 8 bits
Clock Polarity:  Low (CPOL = 0)
Clock Phase   : 1 Edge (CPHA = 0)
Baud Rate     : fPCLK/8 = 12.5 MHz
First Bit     : MSB First
NSS           : Software (GPIO control)
```

#### Lý thuyết SPI

**Đặc điểm:**
- Giao thức đồng bộ (synchronous)
- 4 dây:  SCK (clock), MOSI (data out), MISO (data in), CS (chip select)
- Full-duplex (truyền và nhận đồng thời)
- Tốc độ cao (MHz)
- Master-slave architecture

**Cơ chế hoạt động:**
1. Master kéo CS xuống LOW để chọn slave
2. Master tạo clock trên SCK
3. Data được truyền trên MOSI (Master→Slave) và MISO (Slave→Master) đồng thời
4. Sau khi truyền xong, CS được kéo lên HIGH

**Ưu điểm:**
- Tốc độ cao (lên đến hàng chục MHz)
- Full-duplex
- Đơn giản, không cần địa chỉ
- Không cần pull-up resistors

**Nhược điểm:**
- Cần nhiều dây (mỗi slave cần 1 CS riêng)
- Khoảng cách ngắn
- Không có cơ chế ACK/NACK

#### Code ví dụ - Đọc BME280 qua SPI
```c
// Low-level SPI Read function
HAL_StatusTypeDef BME280_SPI_ReadRegister(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    HAL_StatusTypeDef status;
    
    for (uint8_t i = 0; i < len; i++) {
        // MSB = 1 for read operation
        uint8_t tx_cmd = (reg_addr + i) | 0x80;
        
        // Pull CS low
        HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_RESET);
        
        // Send register address
        status = HAL_SPI_Transmit(&hspi3, &tx_cmd, 1, 100);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
            return status;
        }
        
        // Read single byte
        status = HAL_SPI_Receive(&hspi3, &data[i], 1, 100);
        
        // Pull CS high
        HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
        
        if (status != HAL_OK) return status;
        
        HAL_Delay(1); // Small delay between reads
    }
    
    return HAL_OK;
}

// High-level read function
HAL_StatusTypeDef BME280_ReadAll(BME280_Data_t* data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[8];
    
    // Trigger forced mode measurement
    uint8_t ctrl_meas = 0x25; // Temp x1, Press x1, Forced mode
    status = BME280_SPI_WriteRegister(BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (status != HAL_OK) return status;
    
    HAL_Delay(10); // Wait for measurement
    
    // Read all sensor data (0xF7 to 0xFE = 8 bytes)
    status = BME280_SPI_ReadRegister(BME280_REG_PRESS_MSB, buffer, 8);
    if (status != HAL_OK) return status;
    
    // Parse pressure (20-bit)
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
    
    // Parse temperature (20-bit)
    int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);
    
    // Parse humidity (16-bit)
    int32_t adc_H = ((uint32_t)buffer[6] << 8) | buffer[7];
    
    // Compensate values using calibration data
    int32_t temp_int = BME280_CompensateTemperature(adc_T);
    data->temperature = temp_int / 100.0f;
    
    uint32_t press_int = BME280_CompensatePressure(adc_P);
    data->pressure = press_int / 25600.0f;
    
    uint32_t hum_int = BME280_CompensateHumidity(adc_H);
    data->humidity = hum_int / 1024.0f;
    
    data->valid = true;
    return HAL_OK;
}

// Sử dụng trong SensorReadingTask
void SensorReadingTask(void *argument)
{
    BME280_Data_t bme280_data;
    
    for(;;)
    {
        // Acquire SPI mutex (timeout 1000ms)
        if (osMutexAcquire(spiMutexHandle, 1000) == osOK) 
        {
            // === CRITICAL SECTION - Chỉ 1 task truy cập SPI3 ===
            
            if (BME280_ReadAll(&bme280_data) == HAL_OK) {
                sensorData.temperature = bme280_data.temperature;
                sensorData.humidity = bme280_data.humidity;
                sensorData.pressure = bme280_data.pressure;
                sensorData.bme_valid = 1;
                systemStatus.bme280_connected = true;
            } else {
                sensorData. bme_valid = 0;
                systemStatus.bme280_connected = false;
            }
            
            // Release mutex
            osMutexRelease(spiMutexHandle);
            
            // === END CRITICAL SECTION ===
        }
        
        osDelay(3000); // Read every 3 seconds
    }
}
```

---

### 2.2. I2C2 (Inter-Integrated Circuit)

**Mục đích:** Giao tiếp với cảm biến CCS811 và BH1750

#### Cấu hình phần cứng
```c
// Pin Configuration
PB3  :  I2C2_SDA (Serial Data Line)
PB10 : I2C2_SCL (Serial Clock Line)

// I2C Settings
Mode       : I2C Master
Speed      : 100 kHz (Standard Mode)
Addressing : 7-bit address mode
Pull-up    : Internal pull-up enabled (hoặc external 4.7kΩ)
```

#### Địa chỉ I2C
```c
CCS811  : 0x5A << 1 = 0xB4 (Write), 0xB5 (Read)
BH1750  : 0x23 << 1 = 0x46 (Write), 0x47 (Read)
          (hoặc 0x5C nếu ADDR pin = HIGH)
```

#### Code ví dụ - Đọc CCS811 và BH1750
```c
// Trong SensorReadingTask
void SensorReadingTask(void *argument)
{
    SensorData_t sensorData;
    CCS811_Init();  // Khởi tạo CCS811
    BH1750_Init(BH1750_DEFAULT_ADDRESS);  // Khởi tạo BH1750
    
    for(;;)
    {
        // Acquire I2C mutex (timeout 1000ms)
        if (osMutexAcquire(i2cMutexHandle, 1000) == osOK) 
        {
            // === CRITICAL SECTION - Chỉ 1 task truy cập I2C2 ===
            
            // 1. Đọc CCS811 (CO2 và TVOC)
            if (systemStatus.ccs811_connected) {
                uint16_t co2, tvoc;
                if (CCS811_ReadData(&co2, &tvoc) == HAL_OK) {
                    sensorData.co2 = co2;
                    sensorData.tvoc = tvoc;
                    sensorData. ccs_valid = 1;
                    
                    // Environmental compensation cho CCS811
                    CCS811_SetEnvironmentalData(sensorData.temperature, 
                                                sensorData. humidity);
                } else {
                    sensorData.ccs_valid = 0;
                }
            }
            
            // 2. Đọc BH1750 (Light intensity)
            if (systemStatus.bh1750_connected) {
                float lux;
                if (BH1750_ReadLight(&lux) == BH1750_OK) {
                    sensorData.light_lux = lux;
                    sensorData.bh1750_valid = 1;
                } else {
                    sensorData.bh1750_valid = 0;
                }
            }
            
            // Release mutex
            osMutexRelease(i2cMutexHandle);
            
            // === END CRITICAL SECTION ===
        }
        
        // Gửi dữ liệu vào queue
        osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
        
        osDelay(3000); // 3 seconds
    }
}
```

#### BH1750 - Cảm biến ánh sáng

**Đặc điểm:**
- Dải đo: 1 - 65535 lux
- Độ phân giải: 0.5 lux (High Resolution Mode 2)
- Thời gian đo: 120ms (Continuous mode)
- Nguồn:  3.3V/5V
- Tiêu thụ: 0.12mA (active), 0.01µA (power down)

**API Functions:**
```c
// Khởi tạo cảm biến
BH1750_Status_t BH1750_Init(uint8_t address);

// Đọc cường độ ánh sáng
BH1750_Status_t BH1750_ReadLight(float* lux);

// Set measurement mode
BH1750_Status_t BH1750_SetMode(BH1750_Mode_t mode);
// Modes:  CONTINUOUS_HIGH_RES_MODE, CONTINUOUS_HIGH_RES_MODE_2, 
//        CONTINUOUS_LOW_RES_MODE, ONE_TIME_HIGH_RES_MODE, etc.

// Check connection
bool BH1750_IsConnected(void);

// Power management
BH1750_Status_t BH1750_PowerOn(void);
BH1750_Status_t BH1750_PowerDown(void);
```

**Ví dụ sử dụng:**
```c
// Khởi tạo
if (BH1750_Init(BH1750_DEFAULT_ADDRESS) == BH1750_OK) {
    printf("BH1750 initialized successfully\r\n");
}

// Đọc giá trị ánh sáng
float lux;
if (BH1750_ReadLight(&lux) == BH1750_OK) {
    printf("Light intensity: %.1f lux\r\n", lux);
    
    // Phân loại mức sáng
    if (lux < 10) {
        printf("Dark\r\n");
    } else if (lux < 100) {
        printf("Dim\r\n");
    } else if (lux < 1000) {
        printf("Indoor\r\n");
    } else {
        printf("Bright/Outdoor\r\n");
    }
}
```

---

### 2.3. I2C3 (OLED Display)

**Mục đích:** Hiển thị thông tin trên màn hình OLED SSD1306

#### Cấu hình phần cứng
```c
// Pin Configuration
PB6  : I2C3_SCL (Serial Clock Line)
PB7  : I2C3_SDA (Serial Data Line)

// I2C Settings
Mode       : I2C Master
Speed      : 400 kHz (Fast Mode)
Addressing : 7-bit address mode
OLED Addr  : 0x3C << 1 = 0x78 (Write)
```

#### SSD1306 OLED - Màn hình 128x64

**Đặc điểm:**
- Resolution: 128x64 pixels
- Monochrome (Đen/Trắng)
- Driver IC: SSD1306
- I2C address: 0x3C (default)
- Nguồn: 3.3V - 5V

**API Functions:**
```c
// Khởi tạo OLED
uint8_t SSD1306_Init(void);

// Clear screen
void SSD1306_Fill(SSD1306_COLOR color);
void SSD1306_UpdateScreen(void);

// Vẽ text
void SSD1306_GotoXY(uint8_t x, uint8_t y);
void SSD1306_Puts(char* str, FontDef* font, SSD1306_COLOR color);

// Vẽ hình
void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void SSD1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color);
void SSD1306_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR color);
void SSD1306_DrawCircle(uint8_t x, uint8_t y, uint8_t r, SSD1306_COLOR color);

// Fonts
Font_7x10, Font_11x18, Font_16x26
```

#### Code ví dụ - Hiển thị trên OLED
```c
void OledDisplayTask(void *argument)
{
    SensorData_t sensorData;
    char buffer[32];
    uint8_t page = 0; // Multi-page display
    
    // Khởi tạo OLED
    if (SSD1306_Init() == 1) {
        systemStatus.oled_connected = true;
        
        // Welcome screen
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        SSD1306_GotoXY(2, 2);
        SSD1306_Puts("STM32 Sensor", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(2, 25);
        SSD1306_Puts("Monitor", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_UpdateScreen();
        HAL_Delay(2000);
    }
    
    for(;;)
    {
        // Nhận dữ liệu từ queue
        if (osMessageQueueGet(sensorDataQueueHandle, &sensorData, NULL, 1000) == osOK)
        {
            // Acquire OLED mutex
            if (osMutexAcquire(oledMutexHandle, 1000) == osOK)
            {
                SSD1306_Fill(SSD1306_COLOR_BLACK);
                
                // Hiển thị theo page (rotation mỗi 3 giây)
                switch(page) {
                    case 0: // Page 1:  BME280 data
                        SSD1306_GotoXY(10, 0);
                        SSD1306_Puts("-- BME280 --", &Font_7x10, SSD1306_COLOR_WHITE);
                        
                        if (sensorData.bme_valid) {
                            SSD1306_GotoXY(0, 15);
                            snprintf(buffer, sizeof(buffer), "T: %.1fC", sensorData.temperature);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                            
                            SSD1306_GotoXY(0, 30);
                            snprintf(buffer, sizeof(buffer), "H: %.1f%%", sensorData.humidity);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                            
                            SSD1306_GotoXY(0, 45);
                            snprintf(buffer, sizeof(buffer), "P: %.0fhPa", sensorData.pressure);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                        } else {
                            SSD1306_GotoXY(0, 30);
                            SSD1306_Puts("BME280 ERROR", &Font_7x10, SSD1306_COLOR_WHITE);
                        }
                        break;
                        
                    case 1: // Page 2: CCS811 data
                        SSD1306_GotoXY(10, 0);
                        SSD1306_Puts("-- CCS811 --", &Font_7x10, SSD1306_COLOR_WHITE);
                        
                        if (sensorData.ccs_valid) {
                            SSD1306_GotoXY(0, 20);
                            snprintf(buffer, sizeof(buffer), "CO2: %d ppm", sensorData.co2);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                            
                            SSD1306_GotoXY(0, 35);
                            snprintf(buffer, sizeof(buffer), "TVOC: %d ppb", sensorData.tvoc);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                            
                            // Chất lượng không khí
                            SSD1306_GotoXY(0, 50);
                            if (sensorData.co2 < 600) {
                                SSD1306_Puts("Air:  Excellent", &Font_7x10, SSD1306_COLOR_WHITE);
                            } else if (sensorData.co2 < 1000) {
                                SSD1306_Puts("Air: Good", &Font_7x10, SSD1306_COLOR_WHITE);
                            } else if (sensorData.co2 < 1500) {
                                SSD1306_Puts("Air: Fair", &Font_7x10, SSD1306_COLOR_WHITE);
                            } else {
                                SSD1306_Puts("Air: Poor", &Font_7x10, SSD1306_COLOR_WHITE);
                            }
                        } else {
                            SSD1306_GotoXY(0, 30);
                            SSD1306_Puts("CCS811 ERROR", &Font_7x10, SSD1306_COLOR_WHITE);
                        }
                        break;
                        
                    case 2: // Page 3: BH1750 + Soil Moisture
                        SSD1306_GotoXY(5, 0);
                        SSD1306_Puts("-- BH1750 --", &Font_7x10, SSD1306_COLOR_WHITE);
                        
                        if (sensorData.bh1750_valid) {
                            SSD1306_GotoXY(0, 15);
                            snprintf(buffer, sizeof(buffer), "Light: %.0f lux", sensorData. light_lux);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                        }
                        
                        SSD1306_GotoXY(5, 35);
                        SSD1306_Puts("-- Soil --", &Font_7x10, SSD1306_COLOR_WHITE);
                        
                        if (sensorData.soil_valid) {
                            SSD1306_GotoXY(0, 50);
                            snprintf(buffer, sizeof(buffer), "Moisture: %d%%", sensorData.soil_moisture_percent);
                            SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
                        }
                        break;
                }
                
                // Update screen
                SSD1306_UpdateScreen();
                
                // Release mutex
                osMutexRelease(oledMutexHandle);
                
                // Next page
                page = (page + 1) % 3;
            }
        }
        
        osDelay(3000); // 3 seconds per page
    }
}
```

---

### 2.4. ADC1 (Analog-to-Digital Converter)

**Mục đích:** Đọc cảm biến độ ẩm đất (Soil Moisture Sensor)

#### Cấu hình phần cứng
```c
// Pin Configuration
PA0 : ADC1_IN0 (Soil Moisture Sensor Output)
PA1 : ADC1_IN1 (Reserved for additional analog sensor)

// ADC Settings
Resolution    : 12-bit (0-4095)
Sampling Time :  84 cycles
Conversion Mode: Single conversion
Reference Voltage: 3.3V
```

#### Soil Moisture Sensor

**Nguyên lý:**
- Cảm biến điện trở:  Điện trở thay đổi theo độ ẩm đất
- Output: 0-3.3V analog voltage
- ADC converts:  0V → 0, 3.3V → 4095
- Độ ẩm cao → Điện trở thấp → Voltage cao → ADC value cao

**Calibration:**
```c
#define SOIL_DRY_VALUE      3500  // ADC value khi đất khô (không có nước)
#define SOIL_WET_VALUE      1500  // ADC value khi đất ướt (ngập nước)

uint8_t ConvertSoilMoistureToPercent(uint16_t adc_value)
{
    // Chuyển ADC value → Percent (0% = khô, 100% = ướt)
    if (adc_value >= SOIL_DRY_VALUE) return 0;
    if (adc_value <= SOIL_WET_VALUE) return 100;
    
    int percent = 100 - ((adc_value - SOIL_WET_VALUE) * 100) / (SOIL_DRY_VALUE - SOIL_WET_VALUE);
    
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    return (uint8_t)percent;
}
```

#### Code ví dụ - Đọc Soil Moisture
```c
// Khởi tạo cảm biến độ ẩm đất
void InitializeSoilSensor(void)
{
    // Start ADC calibration
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK) {
        systemStatus.soil_sensor_active = true;
    }
}

// Đọc ADC value
uint16_t ReadSoilMoistureADC(void)
{
    uint16_t adc_value = 0;
    
    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        // Wait for conversion complete (timeout 100ms)
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            // Get ADC value
            adc_value = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    
    return adc_value;
}

// Sử dụng trong SensorReadingTask
void SensorReadingTask(void *argument)
{
    for(;;)
    {
        // ...  đọc các cảm biến khác ...
        
        // Đọc Soil Moisture
        if (systemStatus.soil_sensor_active) {
            uint16_t adc_raw = ReadSoilMoistureADC();
            sensorData.soil_moisture_raw = adc_raw;
            sensorData.soil_moisture_percent = ConvertSoilMoistureToPercent(adc_raw);
            sensorData.soil_valid = 1;
            
            // Cảnh báo khi đất quá khô
            if (sensorData.soil_moisture_percent < 20) {
                // Trigger warning/notification
                printf("WARNING: Soil too dry! Moisture: %d%%\r\n", sensorData.soil_moisture_percent);
            }
        }
        
        osDelay(3000);
    }
}
```

---

### 2.5. UART2 (Universal Asynchronous Receiver-Transmitter)

**Mục đích:** Truyền dữ liệu đến Orange Pi 4A

#### Cấu hình phần cứng
```c
// Pin Configuration
PA2 : USART2_TX (Transmit)
PA3 : USART2_RX (Receive)

// UART Settings
Baud Rate       : 115200 bps
Word Length     : 8 bits
Stop Bits       : 1
Parity          : None
Mode            : Asynchronous
Hardware Flow   : None
```

#### Code ví dụ - Truyền dữ liệu UART
```c
void UartCommunicationTask(void *argument)
{
    SensorData_t sensorData;
    char uartBuffer[512];
    
    for(;;)
    {
        // Nhận dữ liệu từ queue
        if (osMessageQueueGet(sensorDataQueueHandle, &sensorData, NULL, 5000) == osOK)
        {
            // Format dữ liệu thành JSON
            int len = snprintf(uartBuffer, sizeof(uartBuffer),
                "{"
                "\"device\": \"STM32F411_RTOS\","
                "\"timestamp\":%lu,"
                "\"temp\": %.2f,"
                "\"humi\":%.2f,"
                "\"pres\":%.2f,"
                "\"co2\":%d,"
                "\"tvoc\":%d,"
                "\"lux\":%.1f,"
                "\"soil\":%d,"
                "\"bme\":\"%s\","
                "\"ccs\":\"%s\","
                "\"bh1750\":\"%s\","
                "\"soil_sensor\":\"%s\""
                "}\r\n",
                sensorData.timestamp,
                sensorData.temperature,
                sensorData.humidity,
                sensorData.pressure,
                sensorData.co2,
                sensorData.tvoc,
                sensorData.light_lux,
                sensorData.soil_moisture_percent,
                sensorData.bme_valid ? "OK" : "ERROR",
                sensorData.ccs_valid ?  "OK" : "ERROR",
                sensorData.bh1750_valid ? "OK" :  "ERROR",
                sensorData.soil_valid ? "OK" : "ERROR"
            );
            
            // Acquire UART mutex
            if (osMutexAcquire(uartMutexHandle, 1000) == osOK)
            {
                HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, len, 100);
                osMutexRelease(uartMutexHandle);
            }
        }
    }
}
```

---

### 2.6. GPIO (General Purpose Input/Output)

**LED PC13:** Báo trạng thái hệ thống

#### Cấu hình
```c
// Pin: PC13
// Mode: Output Push-Pull
// Active:  LOW (LED sáng khi PC13 = 0)
```

#### Code ví dụ - Điều khiển LED
```c
void LedStatusTask(void *argument)
{
    for(;;)
    {
        // Kiểm tra trạng thái tất cả cảm biến
        uint8_t sensor_ok_count = 0;
        
        if (systemStatus.bme280_connected) sensor_ok_count++;
        if (systemStatus.ccs811_connected) sensor_ok_count++;
        if (systemStatus.bh1750_connected) sensor_ok_count++;
        if (systemStatus.soil_sensor_active) sensor_ok_count++;
        
        if (sensor_ok_count >= 2) {
            // Ít nhất 2 cảm biến OK → LED sáng liên tục
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Active LOW
        } else {
            // Ít hơn 2 cảm biến → LED nhấp nháy cảnh báo
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
        
        osDelay(500); // 500ms
    }
}
```

---

## 3. FREERTOS - TASKS VÀ CƠ CHẾ HOẠT ĐỘNG

### 3.1. Tổng quan về FreeRTOS

**FreeRTOS** là hệ điều hành thời gian thực (RTOS) mã nguồn mở, nhỏ gọn, phù hợp cho vi điều khiển. 

**Đặc điểm:**
- **Preemptive Scheduling**: Task ưu tiên cao có thể chiếm quyền từ task ưu tiên thấp
- **Priority-based**: Mỗi task có mức độ ưu tiên (0 = thấp nhất, 55 = cao nhất)
- **Deterministic**: Thời gian phản hồi có thể dự đoán được
- **Small footprint**: RAM ~2-3KB, Flash ~6-10KB

### 3.2. Cấu hình FreeRTOS

File:  `FreeRTOSConfig.h`

```c
// 1. TICK RATE
#define configTICK_RATE_HZ                       ((TickType_t)1000)
// => 1 tick = 1 ms

// 2. CPU FREQUENCY
#define configCPU_CLOCK_HZ                       ((uint32_t)100000000)
// => STM32F411CE @ 100 MHz

// 3. HEAP SIZE
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
// => 15 KB RAM cho tasks, queues, mutexes

// 4. PRIORITIES
#define configMAX_PRIORITIES                     ( 56 )

// 5. PREEMPTION
#define configUSE_PREEMPTION                     1

// 6. MUTEXES
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1

// 7. HEAP IMPLEMENTATION
#define USE_FreeRTOS_HEAP_4
```

### 3.3. Các Task trong hệ thống

Hệ thống có **5 tasks chính**:

| Task Name             | Priority              | Stack Size | Period      | Mục đích                                      |
|-----------------------|-----------------------|------------|-------------|-----------------------------------------------|
| SensorReadingTask     | osPriorityHigh        | 512 words  | 3000 ms     | Đọc BME280, CCS811, BH1750, Soil Moisture     |
| UartCommunicationTask | osPriorityAboveNormal | 768 words  | Event-driven| Truyền dữ liệu qua UART                       |
| OledDisplayTask       | osPriorityNormal      | 768 words  | Event-driven| Hiển thị dữ liệu trên OLED                    |
| LedStatusTask         | osPriorityBelowNormal | 256 words  | 500 ms      | Điều khiển LED trạng thái                     |
| SystemMonitorTask     | osPriorityLow         | 512 words  | 60000 ms    | Gửi heartbeat, monitor system                 |

---

#### **Task 1: SensorReadingTask (Priority: High)**

**Code:**
```c
void SensorReadingTask(void *argument)
{
    SensorData_t sensorData;
    BME280_Data_t bme280_data;
    
    // Khởi tạo cảm biến
    InitializeSensors();
    
    for(;;)
    {
        memset(&sensorData, 0, sizeof(SensorData_t));
        sensorData.timestamp = HAL_GetTick();
        
        // 1. Đọc BME280 (SPI3)
        if (osMutexAcquire(spiMutexHandle, 1000) == osOK) {
            if (BME280_ReadAll(&bme280_data) == HAL_OK) {
                sensorData.temperature = bme280_data.temperature;
                sensorData.humidity = bme280_data.humidity;
                sensorData.pressure = bme280_data.pressure;
                sensorData.bme_valid = 1;
            }
            osMutexRelease(spiMutexHandle);
        }
        
        // 2. Đọc CCS811 và BH1750 (I2C2)
        if (osMutexAcquire(i2cMutexHandle, 1000) == osOK) {
            // CCS811
            if (systemStatus.ccs811_connected) {
                uint16_t co2, tvoc;
                if (CCS811_ReadData(&co2, &tvoc) == HAL_OK) {
                    sensorData.co2 = co2;
                    sensorData.tvoc = tvoc;
                    sensorData.ccs_valid = 1;
                }
            }
            
            // BH1750
            if (systemStatus.bh1750_connected) {
                float lux;
                if (BH1750_ReadLight(&lux) == BH1750_OK) {
                    sensorData.light_lux = lux;
                    sensorData.bh1750_valid = 1;
                }
            }
            
            osMutexRelease(i2cMutexHandle);
        }
        
        // 3. Đọc Soil Moisture (ADC1)
        if (systemStatus.soil_sensor_active) {
            uint16_t adc_raw = ReadSoilMoistureADC();
            sensorData.soil_moisture_raw = adc_raw;
            sensorData.soil_moisture_percent = ConvertSoilMoistureToPercent(adc_raw);
            sensorData.soil_valid = 1;
        }
        
        // Gửi dữ liệu vào queue
        osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
        
        osDelay(3000); // 3 seconds
    }
}
```

---

## 4. FREERTOS - QUEUE, SEMAPHORE, MUTEX

### 4.1. Queue (Hàng đợi)

**Định nghĩa data structure:**
```c
typedef struct {
    uint32_t timestamp;
    
    // BME280 data
    float temperature;
    float humidity;
    float pressure;
    uint8_t bme_valid;
    
    // CCS811 data
    uint16_t co2;
    uint16_t tvoc;
    uint8_t ccs_valid;
    
    // BH1750 data
    float light_lux;
    uint8_t bh1750_valid;
    
    // Soil Moisture data
    uint16_t soil_moisture_raw;
    uint8_t soil_moisture_percent;
    uint8_t soil_valid;
} SensorData_t;
```

**Tạo queue:**
```c
// Handle
osMessageQueueId_t sensorDataQueueHandle;

// Create queue (5 messages, mỗi message = sizeof(SensorData_t))
sensorDataQueueHandle = osMessageQueueNew(5, sizeof(SensorData_t), NULL);
```

### 4.2. Mutex (Mutual Exclusion)

**Hệ thống sử dụng 4 mutexes:**

```c
osMutexId_t spiMutexHandle;   // Bảo vệ SPI3 (BME280)
osMutexId_t i2cMutexHandle;   // Bảo vệ I2C2 (CCS811, BH1750)
osMutexId_t uartMutexHandle;  // Bảo vệ UART2
osMutexId_t oledMutexHandle;  // Bảo vệ I2C3 (OLED)
```

**Tạo mutex với priority inheritance:**
```c
const osMutexAttr_t mutex_attributes = {
    .attr_bits = osMutexPrioInherit, // Priority inheritance
};

spiMutexHandle = osMutexNew(&mutex_attributes);
i2cMutexHandle = osMutexNew(&mutex_attributes);
uartMutexHandle = osMutexNew(&mutex_attributes);
oledMutexHandle = osMutexNew(&mutex_attributes);
```

---

## 5. CẤU HÌNH HỆ THỐNG

### 5.1. Pin Configuration

```
┌──────────────────────────────────────────────────────────────┐
│             STM32F411CEU6 (Black Pill)                       │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  PC13 ────► LED (Active LOW)                                │
│                                                              │
│  === SPI3 Bus (BME280) ===                                  │
│  PA15 ────► SPI3_NSS/CS ──► BME280 CS                       │
│  PB3  ────► SPI3_SCK    ──► BME280 SCK                      │
│  PB4  ────► SPI3_MISO   ──► BME280 SDO                      │
│  PB5  ────► SPI3_MOSI   ──► BME280 SDI                      │
│                                                              │
│  === I2C2 Bus (CCS811, BH1750) ===                          │
│  PB3  ────► I2C2_SDA ───► CCS811 SDA, BH1750 SDA            │
│  PB10 ────► I2C2_SCL ───► CCS811 SCL, BH1750 SCL            │
│                                                              │
│  === I2C3 Bus (OLED SSD1306) ===                            │
│  PB6  ────► I2C3_SCL ───► OLED SCL                          │
│  PB7  ────► I2C3_SDA ───► OLED SDA                          │
│                                                              │
│  === UART2 (Orange Pi 4A) ===                               │
│  PA2  ────► USART2_TX ──► Orange Pi RX                      │
│  PA3  ────► USART2_RX ◄── Orange Pi TX                      │
│                                                              │
│  === ADC1 (Soil Moisture) ===                               │
│  PA0  ────► ADC1_IN0 ◄─── Soil Moisture Sensor              │
│                                                              │
│  GND  ────► Common Ground                                   │
│  3V3  ────► 3.3V Power                                      │
└──────────────────────────────────────────────────────────────┘
```

### 5.2. Địa chỉ và Protocol Summary

| Device          | Protocol | Address/CS Pin | Data Rate   | Purpose                        |
|-----------------|----------|----------------|-------------|--------------------------------|
| BME280          | SPI3     | PA15 (CS)      | 12. 5 MHz    | Nhiệt độ, độ ẩm, áp suất       |
| CCS811          | I2C2     | 0x5A (0xB4)    | 100 kHz     | CO2, TVOC                      |
| BH1750          | I2C2     | 0x23 (0x46)    | 100 kHz     | Cường độ ánh sáng              |
| SSD1306 OLED    | I2C3     | 0x3C (0x78)    | 400 kHz     | Màn hình hiển thị              |
| Soil Moisture   | ADC1     | PA0 (Channel 0)| 12-bit      | Độ ẩm đất                      |
| Orange Pi 4A    | UART2    | -              | 115200 bps  | Data transmission              |

### 5.3. Memory Usage

**Flash (512 KB):**
```
Application code      : ~95 KB
FreeRTOS kernel       : ~10 KB
HAL drivers           : ~35 KB
Sensor drivers        : ~25 KB (BME280, CCS811, BH1750, OLED)
───────────────────────────────
Total used            : ~165 KB
Free                  : ~347 KB
```

**RAM (128 KB):**
```
Global variables      : ~3 KB
Heap (FreeRTOS)       : 15 KB
  - 5 Tasks           : ~11 KB
  - 1 Queue           : ~0. 5 KB
  - 4 Mutexes         : ~0.4 KB
  - OLED buffer       : ~1 KB (128x64/8 = 1024 bytes)
  - Overhead          : ~0.5 KB
Stack (main)          : ~3 KB
───────────────────────────────
Total used            : ~21 KB
Free                  : ~107 KB
```

---

## 6. HƯỚNG DẪN SỬ DỤNG

### 6.1. Build và Flash

**Yêu cầu:**
- STM32CubeIDE 1.11+
- ST-Link V2 hoặc tương đương

**Các bước:**
```bash
# 1. Clone repository
git clone https://github.com/n22dcdt066-toan/STM32-BME280-CCS811-CBDAD-OLED-PREERTOS.git

# 2. Mở project trong STM32CubeIDE
# File → Open Projects from File System

# 3. Build project
# Project → Build All (Ctrl+B)

# 4. Flash lên STM32F411CEU6
# Run → Debug (F11)
```

### 6.2. Kết nối phần cứng

**BME280 (SPI3):**
```
STM32          BME280
─────          ──────
PA15 (CS)  ──► CS
PB3 (SCK)  ──► SCK
PB4 (MISO) ◄── SDO
PB5 (MOSI) ──► SDI
3V3        ──► VCC
GND        ──► GND
```

**CCS811 (I2C2):**
```
STM32          CCS811
─────          ──────
PB3 (SDA)  ◄─► SDA
PB10 (SCL) ──► SCL
3V3        ──► VCC
GND        ──► GND
GND        ──► WAK (Wake pin)
```

**BH1750 (I2C2):**
```
STM32          BH1750
─────          ──────
PB3 (SDA)  ◄─► SDA
PB10 (SCL) ──► SCL
3V3        ──► VCC
GND        ──► GND
```

**SSD1306 OLED (I2C3):**
```
STM32          OLED
─────          ────
PB6 (SCL)  ──► SCL
PB7 (SDA)  ◄─► SDA
3V3        ──► VCC
GND        ──► GND
```

**Soil Moisture Sensor (ADC1):**
```
STM32          Soil Sensor
─────          ───────────
PA0        ◄── Analog Out
3V3        ──► VCC
GND        ──► GND
```

**Orange Pi 4A (UART2):**
```
STM32          Orange Pi
─────          ─────────
PA2 (TX)   ──► RX
PA3 (RX)   ◄── TX
GND        ──► GND
```

**Lưu ý:**
- I2C2 cần pull-up resistors 4.7kΩ trên SDA và SCL (hoặc dùng internal pull-up)
- I2C3 cần pull-up resistors 4.7kΩ trên SDA và SCL
- BME280 SPI:  CS là active LOW
- CCS811: WAK pin phải nối GND để kích hoạt

### 6.3. Kiểm tra output

**Terminal Serial (115200 baud):**
```bash
# Linux/Mac: 
screen /dev/ttyUSB0 115200

# Python script:
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

**Output mẫu (JSON format):**
```json
{"type":"SYSTEM_START","device":"STM32F411_RTOS"}
{"type":"INIT","bme280": true,"ccs811":true,"bh1750":true,"soil": true,"oled":true}

{"device":"STM32F411_RTOS","timestamp":3000,"temp":25.60,"humi":65.20,"pres":1013.20,"co2":400,"tvoc":25,"lux":320. 5,"soil": 45,"bme":"OK","ccs":"OK","bh1750":"OK","soil_sensor":"OK"}

{"device":"STM32F411_RTOS","timestamp":6000,"temp":25.62,"humi":65.18,"pres":1013.18,"co2":401,"tvoc":26,"lux":318.2,"soil":44,"bme":"OK","ccs":"OK","bh1750":"OK","soil_sensor":"OK"}

{"type":"HEARTBEAT","uptime":60,"bme280":true,"ccs811":true,"bh1750":true,"soil":true,"errors":0}
```

**Màn hình OLED hiển thị:**
- **Page 1 (3s)**: BME280 data (Temp, Humi, Pressure)
- **Page 2 (3s)**: CCS811 data (CO2, TVOC, Air Quality)
- **Page 3 (3s)**: BH1750 (Light) + Soil Moisture
- Rotation: 3 pages x 3 seconds = 9 seconds per cycle

**LED Status:**
- **Sáng liên tục**: Ít nhất 2 cảm biến hoạt động bình thường
- **Nhấp nháy 500ms**: Ít hơn 2 cảm biến hoạt động (warning)

---

## 7. TROUBLESHOOTING

### 7.1. BME280 không hoạt động (SPI3)

**Triệu chứng:** `"bme":"ERROR"`

**Giải pháp:**
1. Kiểm tra kết nối SPI (CS, SCK, MISO, MOSI)
2. Đảm bảo CS (PA15) được control đúng (LOW khi truyền, HIGH khi idle)
3. Kiểm tra SPI mode:  CPOL=0, CPHA=0
4. Đo clock với oscilloscope:  ~12.5 MHz
5. Kiểm tra power supply: 3.3V stable

### 7.2. CCS811/BH1750 không hoạt động (I2C2)

**Triệu chứng:** `"ccs":"ERROR"` hoặc `"bh1750":"ERROR"`

**Giải pháp:**
1. Kiểm tra pull-up resistors 4.7kΩ trên SDA/SCL
2. Scan I2C bus với logic analyzer: 
   - CCS811: 0x5A (0xB4 write)
   - BH1750: 0x23 (0x46 write)
3. Kiểm tra CCS811 WAK pin:  Phải nối GND
4. Thử thay đổi I2C speed:  100kHz → 50kHz
5. Kiểm tra bus conflicts: Chỉ 1 task acquire i2cMutex tại 1 thời điểm

### 7.3. OLED không hiển thị (I2C3)

**Triệu chứng:** Màn hình đen hoặc không init thành công

**Giải pháp:**
1. Kiểm tra kết nối I2C3 (PB6=SCL, PB7=SDA)
2. Kiểm tra địa chỉ OLED:  0x3C hoặc 0x3D
3. Kiểm tra pull-up resistors 4.7kΩ
4
