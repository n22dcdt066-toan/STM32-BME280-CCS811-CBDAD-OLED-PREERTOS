# HỆ THỐNG GIÁM SÁT MÔI TRƯỜNG STM32F411CEU6 + FreeRTOS

**Sinh viên thực hiện:** N22DCDT066 - Toàn  
**Dự án:** Hệ thống đọc dữ liệu cảm biến môi trường sử dụng FreeRTOS trên STM32F411CEU6

---

## MỤC LỤC

1. [Giới thiệu chung về hệ thống](#1-giới-thiệu-chung-về-hệ-thống)
2. [Thiết kế phần cứng](#2-thiết-kế-phần-cứng)
3. [Các giao tiếp ngoại vi](#3-các-giao-tiếp-ngoại-vi)
   - 3.1. [SPI3 (BME280)](#31-spi3-serial-peripheral-interface)
   - 3.2. [I2C2 (CCS811, BH1750)](#32-i2c2-inter-integrated-circuit)
   - 3.3. [I2C3 (OLED SSD1306)](#33-i2c3-oled-display)
   - 3.4. [ADC1 (Soil Moisture)](#34-adc1-analog-to-digital-converter)
   - 3.5. [UART2 (Orange Pi)](#35-uart2-universal-asynchronous-receiver-transmitter)
   - 3.6. [GPIO (LED)](#36-gpio-general-purpose-inputoutput)
4. [FreeRTOS - Tasks và cơ chế hoạt động](#4-freertos---tasks-và-cơ-chế-hoạt-động)
5. [FreeRTOS - Queue, Semaphore, Mutex](#5-freertos---queue-semaphore-mutex)
6. [Cấu hình hệ thống](#6-cấu-hình-hệ-thống)
7. [Hướng dẫn sử dụng](#7-hướng-dẫn-sử-dụng)
8. [Troubleshooting](#8-troubleshooting)
9. [Câu hỏi Viva thường gặp](#9-câu-hỏi-viva-thường-gặp)

---

## 1. GIỚI THIỆU CHUNG VỀ HỆ THỐNG

### 1.1.  Tổng quan

Hệ thống giám sát môi trường sử dụng **STM32F411CEU6 (Black Pill)** kết hợp với **FreeRTOS** để đọc dữ liệu từ nhiều cảm biến, hiển thị trên màn hình OLED và truyền thông tin qua UART đến Orange Pi 4A.

**Mục đích:**
- ✅ Giám sát liên tục các thông số môi trường (nhiệt độ, độ ẩm, áp suất, CO2, TVOC, ánh sáng, độ ẩm đất)
- ✅ Hiển thị thông tin realtime trên màn hình OLED 0.96"
- ✅ Xử lý đa nhiệm (multitasking) hiệu quả với FreeRTOS
- ✅ Truyền dữ liệu thời gian thực qua UART
- ✅ Báo hiệu trạng thái hệ thống qua LED

### 1.2. Phần cứng

**Vi điều khiển:**
- **STM32F411CEU6** (Black Pill)
  - CPU: ARM Cortex-M4 @ 100 MHz
  - Flash: 512 KB
  - RAM: 128 KB
  - FPU:  Có hỗ trợ tính toán số thực

**Cảm biến và thiết bị:**
| Thiết bị | Chức năng | Giao tiếp |
|----------|-----------|-----------|
| **BME280** | Nhiệt độ, Độ ẩm, Áp suất | **SPI3** |
| **CCS811** | CO2, TVOC (Chất lượng không khí) | **I2C2** |
| **BH1750** | Cường độ ánh sáng (Lux) | **I2C2** |
| **Soil Moisture** | Độ ẩm đất | **ADC1** |
| **SSD1306 OLED** | Màn hình hiển thị 128x64 | **I2C3** |
| **LED PC13** | Báo trạng thái | **GPIO** |
| **Orange Pi 4A** | Nhận dữ liệu | **UART2** |

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
    │  PA0       │
    └─────┬──────┘
          │
    ┌─────▼──────┐
    │Soil Moisture│
    │   Sensor    │
    └────────────┘
```

### 1.4. Nguyên lý hoạt động

1. **SensorReadingTask** (Priority: High) đọc dữ liệu từ: 
   - BME280 qua **SPI3** (với SPI mutex protection)
   - CCS811 và BH1750 qua **I2C2** (với I2C mutex protection)
   - Soil Moisture qua **ADC1**

2. Dữ liệu được đưa vào **Queue** (sensorDataQueue)

3. **UartCommunicationTask** (Priority: Above Normal):
   - Lấy dữ liệu từ Queue
   - Format thành JSON/CSV/Plain text
   - Gửi qua UART2 đến Orange Pi 4A (với UART mutex protection)

4. **OledDisplayTask** (Priority: Normal):
   - Lấy dữ liệu từ Queue
   - Hiển thị realtime trên OLED (với OLED mutex protection)
   - Rotation 3 pages:  BME280 → CCS811 → BH1750+Soil

5. **LedStatusTask** (Priority: Below Normal):
   - Cập nhật LED dựa trên trạng thái cảm biến
   - LED sáng:  ≥1 sensor OK
   - LED nhấp nháy:  Không có sensor nào

6. **SystemMonitorTask** (Priority: Low):
   - Gửi heartbeat message mỗi 60 giây
   - Monitor stack usage, errors

---

## 2. THIẾT KẾ PHẦN CỨNG

### 2.1. Sơ đồ kết nối tổng quan

<!-- Thêm ảnh sơ đồ tổng quan nếu có -->
![Hardware Overview](images/hardware_overview.jpg)
*Hình 1: Sơ đồ kết nối tổng quan hệ thống*

### 2.2. STM32F411CEU6 Black Pill

<!-- Thêm ảnh Black Pill pinout -->
![STM32F411CEU6 Pinout](images/stm32f411_pinout.png)
*Hình 2:  Pinout STM32F411CEU6 Black Pill*

**Thông số kỹ thuật:**
- **Core:** ARM Cortex-M4 @ 100 MHz
- **Flash:** 512 KB
- **SRAM:** 128 KB
- **GPIO:** 36 pins
- **ADC:** 1x 12-bit (16 channels)
- **SPI:** 3
- **I2C:** 3
- **USART:** 3
- **Timers:** 11 (16-bit, 32-bit)
- **Package:** LQFP48

### 2.3. Cảm biến BME280 (SPI3)

<!-- Thêm ảnh module BME280 -->
![BME280 Module](images/bme280_module.jpg)
*Hình 3: Module cảm biến BME280*

**Thông số kỹ thuật:**
- **Giao tiếp:** SPI (Mode 0 hoặc Mode 3) / I2C
- **Nguồn:** 1.71V - 3.6V (3.3V)
- **Nhiệt độ:** -40°C đến +85°C (±1°C)
- **Độ ẩm:** 0% - 100% (±3%)
- **Áp suất:** 300 - 1100 hPa (±1 hPa)
- **Tiêu thụ:** 3.6µA @ 1 Hz

**Kết nối:**
```
STM32 PA15 (CS)   → BME280 CS
STM32 PB3  (SCK)  → BME280 SCK
STM32 PB4  (MISO) ← BME280 SDO
STM32 PB5  (MOSI) → BME280 SDI
STM32 3. 3V        → BME280 VCC
STM32 GND         → BME280 GND
```

### 2.4. Cảm biến CCS811 (I2C2)

<!-- Thêm ảnh module CCS811 -->
![CCS811 Module](images/ccs811_module. jpg)
*Hình 4: Module cảm biến CCS811*

**Thông số kỹ thuật:**
- **Giao tiếp:** I2C (Address:  0x5A hoặc 0x5B)
- **Nguồn:** 1.8V - 3.6V (3.3V)
- **CO2:** 400 - 8192 ppm
- **TVOC:** 0 - 1187 ppb
- **Tiêu thụ:** 1.2mA - 46mA (tùy mode)
- **Warm-up time:** 20 phút (48h cho kết quả tốt nhất)

**Kết nối:**
```
STM32 PB3  (SDA)  ↔ CCS811 SDA
STM32 PB10 (SCL)  → CCS811 SCL
STM32 3.3V        → CCS811 VCC
STM32 GND         → CCS811 GND
STM32 GND         → CCS811 WAK (Wake pin - MUST connect to GND)
```

### 2.5. Cảm biến BH1750 (I2C2)

<!-- Thêm ảnh module BH1750 -->
![BH1750 Module](images/bh1750_module.jpg)
*Hình 5: Module cảm biến BH1750*

**Thông số kỹ thuật:**
- **Giao tiếp:** I2C (Address: 0x23 hoặc 0x5C)
- **Nguồn:** 2.4V - 3.6V (3.3V)
- **Dải đo:** 1 - 65535 lux
- **Độ phân giải:** 0.5 lux (High Res Mode 2)
- **Tiêu thụ:** 0.12mA (active), 0.01µA (power down)

**Kết nối:**
```
STM32 PB3  (SDA)  ↔ BH1750 SDA
STM32 PB10 (SCL)  → BH1750 SCL
STM32 3.3V        → BH1750 VCC
STM32 GND         → BH1750 GND
BH1750 ADDR       → GND (address 0x23) hoặc VCC (address 0x5C)
```

### 2.6. Màn hình OLED SSD1306 (I2C3)

<!-- Thêm ảnh OLED -->
![OLED SSD1306](images/oled_ssd1306.jpg)
*Hình 6: Màn hình OLED SSD1306 0.96"*

**Thông số kỹ thuật:**
- **Kích thước:** 0.96 inch
- **Độ phân giải:** 128x64 pixels
- **Màu:** Monochrome (Xanh/Trắng/Vàng xanh)
- **Giao tiếp:** I2C (Address: 0x3C hoặc 0x3D)
- **Nguồn:** 3.3V - 5V
- **Tiêu thụ:** ~20mA

**Kết nối:**
```
STM32 PB6 (SCL)   → OLED SCL
STM32 PB7 (SDA)   ↔ OLED SDA
STM32 3.3V        → OLED VCC
STM32 GND         → OLED GND
```

### 2.7. Cảm biến độ ẩm đất (ADC1)

<!-- Thêm ảnh soil moisture sensor -->
![Soil Moisture Sensor](images/soil_moisture. jpg)
*Hình 7: Cảm biến độ ẩm đất*

**Thông số kỹ thuật:**
- **Loại:** Resistive/Capacitive
- **Output:** Analog 0 - 3.3V
- **Nguồn:** 3.3V - 5V

**Kết nối:**
```
STM32 PA0 (ADC1_IN0) ← Soil Sensor AO (Analog Out)
STM32 3.3V           → Soil Sensor VCC
STM32 GND            → Soil Sensor GND
```

### 2.8. Sơ đồ mạch hoàn chỉnh

<!-- Thêm ảnh sơ đồ mạch chi tiết -->
![Complete Circuit Diagram](images/circuit_diagram.png)
*Hình 8: Sơ đồ mạch chi tiết*

### 2.9. Ảnh thực tế hệ thống

<!-- Thêm ảnh thực tế -->
![Hardware Setup](images/hardware_setup.jpg)
*Hình 9: Hệ thống hoàn chỉnh*

---

## 3. CÁC GIAO TIẾP NGOẠI VI

### 3.1. SPI3 (Serial Peripheral Interface)

**Mục đích:** Giao tiếp với cảm biến BME280

#### 3.1.1. Cấu hình phần cứng

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

#### 3.1.2. Lý thuyết SPI

**Đặc điểm:**
- **Giao thức đồng bộ** (synchronous)
- **4 dây:** SCK (clock), MOSI (data out), MISO (data in), CS (chip select)
- **Full-duplex:** Truyền và nhận đồng thời
- **Tốc độ cao:** MHz (hàng chục MHz)
- **Master-slave architecture**

**Cơ chế hoạt động:**
```
Master                           Slave (BME280)
  │                                   │
  │  1. CS = LOW (Select slave)      │
  ├──────────────────────────────────►│
  │                                   │
  │  2. Clock pulses on SCK           │
  │  ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐│
  │ ─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └┤
  │                                   │
  │  3. Data on MOSI (Master→Slave)  │
  ├───[B7][B6][B5][B4][B3][B2][B1][B0]►
  │                                   │
  │  4. Data on MISO (Slave→Master)  │
  ◄───[B7][B6][B5][B4][B3][B2][B1][B0]┤
  │                                   │
  │  5. CS = HIGH (Deselect)          │
  ├──────────────────────────────────►│
  │                                   │
```

**SPI Modes:**
| Mode | CPOL | CPHA | Description |
|------|------|------|-------------|
| 0    | 0    | 0    | Clock idle LOW, sample on rising edge |
| 1    | 0    | 1    | Clock idle LOW, sample on falling edge |
| 2    | 1    | 0    | Clock idle HIGH, sample on falling edge |
| 3    | 1    | 1    | Clock idle HIGH, sample on rising edge |

**BME280 sử dụng Mode 0 hoặc Mode 3**

**Ưu điểm:**
- ✅ Tốc độ cao (lên đến hàng chục MHz)
- ✅ Full-duplex
- ✅ Đơn giản, không cần địa chỉ
- ✅ Không cần pull-up resistors

**Nhược điểm:**
- ❌ Cần nhiều dây (mỗi slave cần 1 CS riêng)
- ❌ Khoảng cách ngắn
- ❌ Không có cơ chế ACK/NACK

#### 3.1.3. Code ví dụ - Đọc BME280 qua SPI (từ Core/Src/BME280.c)

**Low-level SPI Read:**

```c
/**
  * @brief  SPI Read Register
  * @param  reg_addr: Register address
  * @param  data:  Pointer to data buffer
  * @param  len: Number of bytes to read
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_SPI_ReadRegister(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    HAL_StatusTypeDef status;
    
    for (uint8_t i = 0; i < len; i++) {
        // MSB = 1 for read operation (BME280 protocol)
        uint8_t tx_cmd = (reg_addr + i) | 0x80;
        
        // Pull CS low (Select BME280)
        HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_RESET);
        
        // Send register address
        status = HAL_SPI_Transmit(&hspi3, &tx_cmd, 1, BME280_SPI_TIMEOUT);
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
            return status;
        }
        
        // Read single byte
        status = HAL_SPI_Receive(&hspi3, &data[i], 1, BME280_SPI_TIMEOUT);
        
        // Pull CS high (Deselect BME280)
        HAL_GPIO_WritePin(BME280_CS_GPIO_Port, BME280_CS_Pin, GPIO_PIN_SET);
        
        if (status != HAL_OK) return status;
        
        HAL_Delay(1); // Small delay between reads
    }
    
    return HAL_OK;
}
```

**High-level Read All Sensor Data:**

```c
/**
  * @brief  Read all sensor data (temperature, humidity, pressure)
  * @param  data: Pointer to BME280_Data_t structure
  * @retval HAL status
  */
HAL_StatusTypeDef BME280_ReadAll(BME280_Data_t* data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[8];
    
    // Trigger forced mode measurement
    uint8_t ctrl_meas = 0x25; // Temp x1, Press x1, Forced mode
    status = BME280_SPI_WriteRegister(BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (status != HAL_OK) return status;
    
    HAL_Delay(10); // Wait for measurement (~10ms)
    
    // Read all sensor data (0xF7 to 0xFE = 8 bytes)
    // 0xF7-0xF9:  Pressure
    // 0xFA-0xFC: Temperature
    // 0xFD-0xFE: Humidity
    status = BME280_SPI_ReadRegister(BME280_REG_PRESS_MSB, buffer, 8);
    if (status != HAL_OK) return status;
    
    // Parse pressure (20-bit)
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | 
                    ((uint32_t)buffer[1] << 4) | 
                    ((buffer[2] >> 4) & 0x0F);
    
    // Parse temperature (20-bit)
    int32_t adc_T = ((uint32_t)buffer[3] << 12) | 
                    ((uint32_t)buffer[4] << 4) | 
                    ((buffer[5] >> 4) & 0x0F);
    
    // Parse humidity (16-bit)
    int32_t adc_H = ((uint32_t)buffer[6] << 8) | buffer[7];
    
    // Compensate values using calibration data
    int32_t temp_int = BME280_CompensateTemperature(adc_T);
    data->temperature = temp_int / 100.0f; // Convert to °C
    
    uint32_t press_int = BME280_CompensatePressure(adc_P);
    data->pressure = press_int / 25600.0f; // Convert to hPa
    
    uint32_t hum_int = BME280_CompensateHumidity(adc_H);
    data->humidity = hum_int / 1024.0f; // Convert to %
    
    data->valid = true;
    return HAL_OK;
}
```

---

### 3.2. I2C2 (Inter-Integrated Circuit)

**Mục đích:** Giao tiếp với cảm biến CCS811 và BH1750

#### 3.2.1. Cấu hình phần cứng

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

**Địa chỉ I2C:**
```c
CCS811  : 0x5A << 1 = 0xB4 (Write), 0xB5 (Read)
          0x5B << 1 = 0xB6 (Write), 0xB7 (Read) - secondary
BH1750  : 0x23 << 1 = 0x46 (Write), 0x47 (Read)
          0x5C << 1 = 0xB8 (Write), 0xB9 (Read) - nếu ADDR pin = HIGH
```

#### 3.2.2. Lý thuyết I2C

**Đặc điểm:**
- **Giao thức đồng bộ** (synchronous)
- **2 dây:** SDA (data), SCL (clock)
- **Multi-master, multi-slave**
- **Tốc độ:** 100 kHz (Standard), 400 kHz (Fast), 3. 4 MHz (High-speed)

**Cơ chế hoạt động:**
```
Master                                    Slave
  │                                          │
  │  1. START condition (S)                  │
  │     SDA:  HIGH → LOW while SCL = HIGH     │
  ├─────────────────────────────────────────►│
  │                                          │
  │  2. Address + R/W bit (7-bit + 1-bit)    │
  │     [A6][A5][A4][A3][A2][A1][A0][R/W]   │
  ├─────────────────────────────────────────►│
  │                                          │
  │  3. ACK from slave                       │
  ◄─────────────────────────────────────────┤
  │                                          │
  │  4. Data byte (8-bit)                    │
  │     [D7][D6][D5][D4][D3][D2][D1][D0]    │
  ├─────────────────────────────────────────►│
  │                                          │
  │  5. ACK from slave                       │
  ◄─────────────────────────────────────────┤
  │                                          │
  │  ...  Repeat steps 4-5 for more data ...  │
  │                                          │
  │  6. STOP condition (P)                   │
  │     SDA: LOW → HIGH while SCL = HIGH     │
  ├─────────────────────────────────────────►│
  │                                          │
```

**START & STOP Conditions:**
```
START (S):             STOP (P):
    SCL ─────┐         SCL ─────┐
             │                  │
    SDA ───┐ │         SDA    ┌─┘
           │ │                │
           └─┘                └───
```

**ACK/NACK:**
- **ACK (0):** Slave pulls SDA LOW → Data received successfully
- **NACK (1):** Slave releases SDA (HIGH) → No more data / Error

**Ưu điểm:**
- ✅ Chỉ cần 2 dây
- ✅ Hỗ trợ nhiều slave (multi-device)
- ✅ Có cơ chế ACK/NACK (error detection)
- ✅ Addressing system (mỗi slave có địa chỉ riêng)

**Nhược điểm:**
- ❌ Tốc độ thấp hơn SPI
- ❌ Cần pull-up resistors (4.7kΩ)
- ❌ Phức tạp hơn UART

#### 3.2.3. CCS811 - Cảm biến chất lượng không khí

**Đặc điểm:**
- Dải đo CO2: 400 - 8192 ppm
- Dải đo TVOC: 0 - 1187 ppb
- Measurement modes: Idle, 1s, 10s, 60s, 250ms
- Environmental compensation: Hỗ trợ (từ nhiệt độ & độ ẩm)

**Registers quan trọng:**
```c
// CCS811 Register Definitions (từ Core/Inc/CCS811.h)
#define CCS811_STATUS           0x00  // Status register
#define CCS811_MEAS_MODE        0x01  // Measurement mode
#define CCS811_ALG_RESULT_DATA  0x02  // Algorithm result (CO2, TVOC)
#define CCS811_RAW_DATA         0x03  // Raw ADC data
#define CCS811_ENV_DATA         0x05  // Environmental data (temp, humi)
#define CCS811_HW_ID            0x20  // Hardware ID (should be 0x81)
#define CCS811_ERROR_ID         0xE0  // Error ID
#define CCS811_SW_RESET         0xFF  // Software reset

// Status bits
#define CCS811_STATUS_FW_MODE           0x80  // Firmware mode (0=boot, 1=app)
#define CCS811_STATUS_APP_VALID         0x10  // Application valid
#define CCS811_STATUS_DATA_READY        0x08  // Data ready to read
#define CCS811_STATUS_ERROR             0x01  // Error occurred

// Measurement modes
#define CCS811_DRIVE_MODE_IDLE          0x00  // Idle mode
#define CCS811_DRIVE_MODE_1SEC          0x10  // 1 measurement per second
#define CCS811_DRIVE_MODE_10SEC         0x20  // 1 measurement per 10 seconds
#define CCS811_DRIVE_MODE_60SEC         0x30  // 1 measurement per 60 seconds
#define CCS811_DRIVE_MODE_250MS         0x40  // 1 measurement per 250ms
```

**Data Structure:**
```c
// CCS811 Data Structure (từ Core/Inc/CCS811.h)
typedef struct {
    uint16_t co2;        // eCO2 in ppm (Equivalent CO2)
    uint16_t tvoc;       // TVOC in ppb (Total Volatile Organic Compounds)
    uint8_t status;      // Status register value
    uint8_t error_id;    // Error ID if error occurred
    uint16_t raw_data;   // Raw ADC data
    uint8_t current;     // Current through sensor (µA)
    uint16_t raw_voltage;// Raw voltage across sensor
    bool valid;          // Data validity flag
} CCS811_Data_t;
```

**Code ví dụ - CCS811 Implementation (từ Core/Src/CCS811_Basic.c):**

**1. Khởi tạo CCS811:**

```c
/**
  * @brief  Initialize CCS811 sensor
  * @retval HAL status
  */
HAL_StatusTypeDef CCS811_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t hw_id, hw_version;
    uint8_t status_reg;
    
    // Try primary address first (0x5A)
    ccs811_i2c_address = CCS811_I2C_ADDR_PRIM;  // 0x5A << 1 = 0xB4
    status = CCS811_ReadRegister(CCS811_HW_ID, &hw_id);
    
    if (status != HAL_OK || hw_id != CCS811_HW_ID_CODE) {
        // Try secondary address (0x5B)
        ccs811_i2c_address = CCS811_I2C_ADDR_SEC;  // 0x5B << 1 = 0xB6
        status = CCS811_ReadRegister(CCS811_HW_ID, &hw_id);
        
        if (status != HAL_OK || hw_id != CCS811_HW_ID_CODE) {
            return HAL_ERROR;  // Sensor not found
        }
    }
    
    // Read hardware version
    status = CCS811_ReadRegister(CCS811_HW_VERSION, &hw_version);
    if (status != HAL_OK) return status;
    
    // Check if application is valid
    status = CCS811_ReadRegister(CCS811_STATUS, &status_reg);
    if (status != HAL_OK) return status;
    
    if (!(status_reg & CCS811_STATUS_APP_VALID)) {
        return HAL_ERROR;  // Application not valid
    }
    
    // Start application if not already started
    if (!(status_reg & CCS811_STATUS_FW_MODE)) {
        status = CCS811_StartApp();
        if (status != HAL_OK) return status;
        
        HAL_Delay(10);  // Wait for app to start
        
        // Verify app started
        status = CCS811_ReadRegister(CCS811_STATUS, &status_reg);
        if (status != HAL_OK) return status;
        
        if (!(status_reg & CCS811_STATUS_FW_MODE)) {
            return HAL_ERROR;  // Failed to start app
        }
    }
    
    ccs811_app_started = true;
    
    // Set drive mode to 1 second intervals
    status = CCS811_SetDriveMode(CCS811_DRIVE_MODE_1SEC);
    if (status != HAL_OK) return status;
    
    return HAL_OK;
}
```

**2. Đọc dữ liệu CO2 và TVOC:**

```c
/**
  * @brief  Read sensor data (CO2, TVOC, status, raw data)
  * @param  data: Pointer to CCS811_Data_t structure
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
    
    // Read algorithm result data (8 bytes from 0x02)
    // Byte 0-1: eCO2 (MSB first)
    // Byte 2-3: TVOC (MSB first)
    // Byte 4:   Status
    // Byte 5:   Error ID
    // Byte 6-7: Raw data (MSB first)
    status = CCS811_ReadRegisters(CCS811_ALG_RESULT_DATA, raw_data, 8);
    if (status != HAL_OK) return status;
    
    // Parse data
    data->co2 = (raw_data[0] << 8) | raw_data[1];        // eCO2 (ppm)
    data->tvoc = (raw_data[2] << 8) | raw_data[3];       // TVOC (ppb)
    data->status = raw_data[4];                          // Status register
    data->error_id = raw_data[5];                        // Error ID
    data->raw_data = (raw_data[6] << 8) | raw_data[7];  // Raw ADC data
    
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
```

**3. Environmental Compensation (Quan trọng!):**

```c
/**
  * @brief  Set environmental data for compensation
  * @param  humidity: Relative humidity (0-100%)
  * @param  temperature: Temperature in Celsius
  * @retval HAL status
  * @note   Formula: 
  *         Humidity = ((humidity % + 0.5) / 100) * 65535
  *         Temperature = (temperature + 25) * 512
  */
HAL_StatusTypeDef CCS811_SetEnvironmentalData(float humidity, float temperature)
{
    if (!ccs811_app_started) return HAL_ERROR;
    
    // Convert humidity to CCS811 format
    uint16_t hum_data = (uint16_t)((humidity + 0.5f) / 100.0f * 65535.0f);
    
    // Convert temperature to CCS811 format  
    uint16_t temp_data = (uint16_t)((temperature + 25.0f) * 512.0f);
    
    // Pack data into 4-byte array (MSB first)
    uint8_t env_data[4];
    env_data[0] = (hum_data >> 8) & 0xFF;   // Humidity MSB
    env_data[1] = hum_data & 0xFF;          // Humidity LSB  
    env_data[2] = (temp_data >> 8) & 0xFF;  // Temperature MSB
    env_data[3] = temp_data & 0xFF;         // Temperature LSB
    
    // Write to ENV_DATA register (0x05)
    return HAL_I2C_Mem_Write(&CCS811_I2C_HANDLE, 
                             ccs811_i2c_address,
                             CCS811_ENV_DATA, 1, 
                             env_data, 4, 
                             CCS811_I2C_TIMEOUT);
}
```

#### 3.2.4. BH1750 - Cảm biến ánh sáng

**Code ví dụ (từ Core/Src/BH1750.c):**

```c
/**
  * @brief  Initialize BH1750 sensor
  * @param  address: I2C address (BH1750_ADDRESS_LOW or BH1750_ADDRESS_HIGH)
  * @retval BH1750_Status_t
  */
BH1750_Status_t BH1750_Init(uint8_t address)
{
    BH1750_Status_t status;
    
    // Set device address (0x23 or 0x5C)
    bh1750_address = (address == BH1750_ADDRESS_HIGH) ? 
                      BH1750_ADDRESS_HIGH : BH1750_ADDRESS_LOW;
    
    // Reset the sensor
    status = BH1750_Reset();
    if (status != BH1750_OK) return status;
    
    HAL_Delay(10);  // Wait for reset
    
    // Power on the sensor
    status = BH1750_PowerOn();
    if (status != BH1750_OK) return status;
    
    // Set measurement mode (High Resolution Mode 2 - 0. 5 lux resolution)
    status = BH1750_SetMode(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
    if (status != BH1750_OK) return status;
    
    HAL_Delay(180);  // Wait for first measurement (120ms + margin)
    
    device_initialized = true;
    return BH1750_OK;
}

/**
  * @brief  Read light intensity from BH1750
  * @param  lux:  Pointer to store light intensity value
  * @retval BH1750_Status_t
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
    
    // Convert to lux (Resolution 0.5 lux for mode 2)
    *lux = raw_value / 1.2f;
    
    return BH1750_OK;
}
```

---

### 3. 3. I2C3 (OLED Display)

**Mục đích:** Hiển thị thông tin trên màn hình OLED SSD1306

#### 3.3.1. Cấu hình phần cứng

```c
// Pin Configuration
PB6 :  I2C3_SCL (Serial Clock Line)
PB7 : I2C3_SDA (Serial Data Line)

// I2C Settings
Mode       : I2C Master
Speed      : 400 kHz (Fast Mode)
Addressing : 7-bit address mode
OLED Addr  : 0x3C << 1 = 0x78 (Write)
```

#### 3.3.2. SSD1306 OLED - Màn hình 128x64

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

// Fonts
Font_7x10, Font_11x18, Font_16x26
```

#### 3.3.3. Code ví dụ - Hiển thị trên OLED

```c
/**
  * @brief  OLED Display Task - Updates OLED screen with sensor data
  * @param  argument: Not used
  * @retval None
  */
void OledDisplayTask(void *argument)
{
  SensorData_t sensorData;
  char buffer[32];
  uint8_t page = 0; // Page rotation:  0=BME280, 1=CCS811, 2=BH1750+Soil
  
  // Initialize OLED
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
    // Receive sensor data from queue
    if (osMessageQueueGet(sensorDataQueueHandle, &sensorData, NULL, 1000) == osOK)
    {
      // Acquire OLED mutex
      if (osMutexAcquire(oledMutexHandle, 1000) == osOK)
      {
        SSD1306_Fill(SSD1306_COLOR_BLACK);
        
        // Display based on current page
        switch(page) {
          case 0: // Page 1: BME280 data
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
              snprintf(buffer, sizeof(buffer), "P: %.0fhPa", sensorData. pressure);
              SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
            }
            break;
            
          case 1: // Page 2: CCS811 data
            SSD1306_GotoXY(10, 0);
            SSD1306_Puts("-- CCS811 --", &Font_7x10, SSD1306_COLOR_WHITE);
            
            if (sensorData.ccs_valid) {
              SSD1306_GotoXY(0, 20);
              snprintf(buffer, sizeof(buffer), "CO2: %d ppm", sensorData. co2);
              SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
              
              SSD1306_GotoXY(0, 35);
              snprintf(buffer, sizeof(buffer), "TVOC: %d ppb", sensorData.tvoc);
              SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
              
              // Air quality indicator
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
            }
            break;
            
          case 2: // Page 3: BH1750 + Soil
            SSD1306_GotoXY(5, 0);
            SSD1306_Puts("-- BH1750 --", &Font_7x10, SSD1306_COLOR_WHITE);
            
            if (sensorData.bh1750_valid) {
              SSD1306_GotoXY(0, 15);
              snprintf(buffer, sizeof(buffer), "Light: %.0f lux", sensorData.light_lux);
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
    
    // Delay 3 seconds per page
    osDelay(3000);
  }
}
```

---

### 3.4. ADC1 (Analog-to-Digital Converter)

**Mục đích:** Đọc cảm biến độ ẩm đất

#### 3.4.1. Cấu hình phần cứng

```c
// Pin Configuration
PA0 : ADC1_IN0 (Soil Moisture Sensor Output)

// ADC Settings
Resolution    : 12-bit (0-4095)
Sampling Time :  84 cycles
Conversion Mode:  Single conversion
Reference Voltage: 3.3V
```

#### 3.4.2. Lý thuyết ADC

**Công thức chuyển đổi:**
```
Voltage = (ADC_Value / 4095) × VREF
        = (ADC_Value / 4095) × 3.3V

Ví dụ: 
- ADC = 0    → 0V
- ADC = 2048 → 1.65V  
- ADC = 4095 → 3.3V
```

**Resolution vs Accuracy:**
- **12-bit resolution:** 4096 levels (0-4095)
- **Step size:** 3.3V / 4096 = 0.806mV
- **Accuracy:** Phụ thuộc vào nhiễu, VREF stability, sensor quality

#### 3.4.3. Code ví dụ - Đọc Soil Moisture

```c
#define SOIL_DRY_VALUE   3500  // ADC value khi đất khô (không có nước)
#define SOIL_WET_VALUE   1500  // ADC value khi đất ướt (ngập nước)

/**
  * @brief  Initialize soil moisture sensor
  * @retval None
  */
void InitializeSoilSensor(void)
{
    // Start ADC calibration
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK) {
        systemStatus.soil_sensor_active = true;
    }
}

/**
  * @brief  Read soil moisture ADC value
  * @retval ADC value (0-4095)
  */
uint16_t ReadSoilMoistureADC(void)
{
    uint16_t adc_value = 0;
    
    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        // Wait for conversion (timeout 100ms)
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            adc_value = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    
    return adc_value;
}

/**
  * @brief  Convert ADC value to moisture percentage
  * @param  adc_value: Raw ADC value
  * @retval Moisture percentage (0-100%)
  */
uint8_t ConvertSoilMoistureToPercent(uint16_t adc_value)
{
    // 0% = khô (SOIL_DRY_VALUE), 100% = ướt (SOIL_WET_VALUE)
    if (adc_value >= SOIL_DRY_VALUE) return 0;
    if (adc_value <= SOIL_WET_VALUE) return 100;
    
    int percent = 100 - ((adc_value - SOIL_WET_VALUE) * 100) / 
                        (SOIL_DRY_VALUE - SOIL_WET_VALUE);
    
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    return (uint8_t)percent;
}
```

**Calibration:**
```c
// Để calibrate sensor: 
// 1. Đo ADC khi sensor khô hoàn toàn → SOIL_DRY_VALUE
// 2. Đo ADC khi sensor ngập trong nước → SOIL_WET_VALUE
// 3. Update #define values

// Ví dụ: 
// Đất khô: ADC = 3500 → 0% moisture
// Đất ướt: ADC = 1500 → 100% moisture
// Đất vừa: ADC = 2500 → 50% moisture
```

---

### 3.5.  UART2 (Universal Asynchronous Receiver-Transmitter)

**Mục đích:** Truyền dữ liệu đến Orange Pi 4A

#### 3.5.1. Cấu hình phần cứng

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

#### 3.5.2. Lý thuyết UART

**Đặc điểm:**
- **Giao thức bất đồng bộ** (asynchronous) - không cần clock chung
- **2 dây:** TX, RX (+ GND)
- **Full-duplex:** Truyền và nhận đồng thời
- **Point-to-point:** 1-1

**Frame format (8N1):**
```
┌─────┬───┬───┬───┬───┬───┬───┬───┬───┬──────┐
│START│ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │STOP  │
│  0  │   │   │   DATA BITS   │   │   │  1   │
└─────┴───┴───┴───┴───┴───┴───┴───┴───┴──────┘
  1bit  ←──────── 8 bits ──────────→  1bit

Idle:  HIGH
Start bit: LOW
Data bits: LSB first
Stop bit: HIGH
```

**Tính baud rate:**
```c
Baud Rate = fPCLK / (16 × USARTDIV)

Ví dụ:  115200 = 100MHz / (16 × 54. 25)
```

#### 3.5.3. Code ví dụ - Truyền dữ liệu UART

```c
/**
  * @brief  UART Communication Task
  */
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
        sensorData. light_lux,
        sensorData.soil_moisture_percent,
        sensorData.bme_valid ? "OK" : "ERROR",
        sensorData.ccs_valid ? "OK" :  "ERROR",
        sensorData.bh1750_valid ?  "OK" : "ERROR",
        sensorData. 
