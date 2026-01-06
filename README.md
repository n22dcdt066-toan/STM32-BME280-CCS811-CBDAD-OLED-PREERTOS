# HỆ THỐNG GIÁM SÁT MÔI TRƯỜNG STM32F411CEU6 + FreeRTOS

**Sinh viên thực hiện:** N22DCDT066 - Toàn  
**Nhóm:** 8  
**Dự án:** Hệ thống đọc dữ liệu cảm biến môi trường sử dụng FreeRTOS trên STM32F411CEU6

---

## MỤC LỤC

1. [Giới thiệu chung về hệ thống](#1-giới-thiệu-chung-về-hệ-thống)
2. [Các giao tiếp ngoại vi trong bài](#2-các-giao-tiếp-ngoại-vi-trong-bài)
3. [FreeRTOS - Tasks và cơ chế hoạt động](#3-freertos---tasks-và-cơ-chế-hoạt-động)
4. [FreeRTOS - Queue, Semaphore, Mutex](#4-freertos---queue-semaphore-mutex)
5. [Code trong dự án](#5-code-trong-dự-án)
6. [Câu hỏi Viva thường gặp](#6-câu-hỏi-viva-thường-gặp)

---

## 1. GIỚI THIỆU CHUNG VỀ HỆ THỐNG

### 1.1. Tổng quan dự án

Hệ thống giám sát môi trường sử dụng **STM32F411CEU6 (Black Pill)** kết hợp với **FreeRTOS** để đọc dữ liệu từ nhiều cảm biến, hiển thị trên màn hình OLED và truyền thông tin qua UART đến Orange Pi 4A.

**Mục đích:**
- ✅ Giám sát liên tục các thông số môi trường (nhiệt độ, độ ẩm, áp suất, CO2, TVOC, ánh sáng, độ ẩm đất)
- ✅ Hiển thị thông tin realtime trên màn hình OLED 0.96"
- ✅ Xử lý đa nhiệm (multitasking) hiệu quả với FreeRTOS
- ✅ Truyền dữ liệu thời gian thực qua UART
- ✅ Báo hiệu trạng thái hệ thống qua LED

### 1.2. Phần cứng sử dụng

**Vi điều khiển:**
- **STM32F411CEU6** (Black Pill)
  - CPU: ARM Cortex-M4 @ 100 MHz
  - Flash: 512 KB
  - RAM: 128 KB
  - FPU: Có hỗ trợ tính toán số thực

**Cảm biến và thiết bị ngoại vi:**

| Thiết bị | Chức năng | Giao tiếp | Địa chỉ/Kênh |
|----------|-----------|-----------|--------------|
| **BME280** | Nhiệt độ, Độ ẩm, Áp suất | **SPI3** | CS: PB0 |
| **CCS811** | CO2, TVOC (Chất lượng không khí) | **I2C2** | 0x5A/0x5B |
| **BH1750** | Cường độ ánh sáng (Lux) | **I2C2** | 0x23/0x5C |
| **Soil Moisture** | Độ ẩm đất | **ADC1** | Channel 4 (PA4) |
| **SSD1306 OLED** | Màn hình hiển thị 128x64 | **I2C3** | 0x3C/0x3D |
| **LED PC13** | Báo trạng thái | **GPIO** | Active LOW |
| **Orange Pi 4A** | Nhận dữ liệu | **UART2** | 115200 baud |

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
│  SystemMonitor Task (Priority: Low) - Heartbeat & Monitor          │
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
    │Soil Sensor │
    │   (ADC1)   │
    └────────────┘
```

### 1.4. Nguyên lý hoạt động

1. **SensorReadingTask** đọc dữ liệu từ BME280 (qua SPI3), CCS811 & BH1750 (qua I2C2), và Soil Moisture (qua ADC1)
2. Dữ liệu được đưa vào **sensorDataQueue** (thread-safe communication)
3. **UartCommunicationTask** lấy dữ liệu từ Queue, format thành JSON/CSV/Plain text
4. Dữ liệu được gửi qua UART2 đến Orange Pi 4A (với UART mutex protection)
5. **OledDisplayTask** hiển thị dữ liệu lên màn hình OLED (với OLED mutex protection)
6. **LedStatusTask** cập nhật trạng thái LED dựa trên tình trạng cảm biến
7. **SystemMonitorTask** gửi heartbeat và giám sát hệ thống

---

## 2. CÁC GIAO TIẾP NGOẠI VI TRONG BÀI

### 2.1. SPI3 (Serial Peripheral Interface)

**Lý thuyết:**
- SPI là giao thức đồng bộ, full-duplex, master-slave
- Tốc độ cao (có thể lên đến MHz)
- Sử dụng 4 dây: MOSI, MISO, SCK, CS (Chip Select)

**Trong dự án này:**
- **Thiết bị:** BME280 (cảm biến nhiệt độ, độ ẩm, áp suất)
- **Chân kết nối:**
  ```
  PB5  → SPI3_MOSI  (Master Out Slave In)
  PB4  → SPI3_MISO  (Master In Slave Out)
  PB3  → SPI3_SCK   (Serial Clock)
  PB0  → CS         (Chip Select - GPIO)
  ```
- **Cấu hình:** 
  - Mode: Mode 0 (CPOL=0, CPHA=0)
  - Prescaler: 32 (Clock ~ 3 MHz)
  - Data size: 8-bit
  - MSB first

**Code ví dụ trong dự án:**
```c
// File: Core/Src/BME280.c
uint8_t BME280_ReadRegister(uint8_t reg)
{
    uint8_t txData[2], rxData[2];
    txData[0] = reg | 0x80;  // Set read bit
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // CS LOW
    HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // CS HIGH
    
    return rxData[1];
}
```

**Ưu điểm:** Tốc độ cao, không có địa chỉ xung đột  
**Nhược điểm:** Cần nhiều chân hơn I2C

---

### 2.2. I2C2 (Inter-Integrated Circuit)

**Lý thuyết:**
- I2C là giao thức đồng bộ, half-duplex, multi-master
- Chỉ cần 2 dây: SDA (Data), SCL (Clock)
- Sử dụng địa chỉ 7-bit hoặc 10-bit để phân biệt thiết bị

**Trong dự án này:**
- **Thiết bị:** CCS811 (CO2, TVOC), BH1750 (ánh sáng)
- **Chân kết nối:**
  ```
  PB10 → I2C2_SCL  (Serial Clock)
  PB3  → I2C2_SDA  (Serial Data)
  ```
- **Địa chỉ:**
  - CCS811: 0x5A (primary) hoặc 0x5B
  - BH1750: 0x23 (primary) hoặc 0x5C
- **Cấu hình:**
  - Speed: 100 kHz (Standard Mode)
  - Addressing mode: 7-bit
  - Timeout: 1000 ms

**Code ví dụ trong dự án:**
```c
// File: Core/Src/CCS811_Basic.c
#define CCS811_ADDR (0x5A << 1)  // HAL uses 8-bit address (shifted)

uint8_t CCS811_ReadRegister(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDR, reg, 
                     I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    return data;
}

void CCS811_WriteRegister(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c2, CCS811_ADDR, reg, 
                      I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}
```

**Ưu điểm:** Tiết kiệm chân, nhiều thiết bị trên cùng bus  
**Nhược điểm:** Tốc độ thấp hơn SPI, dễ bị xung đột địa chỉ

---

### 2.3. I2C3 (OLED Display)

**Lý thuyết:** Giống I2C2, nhưng sử dụng bus riêng để tránh xung đột với cảm biến

**Trong dự án này:**
- **Thiết bị:** SSD1306 OLED 128x64
- **Chân kết nối:**
  ```
  PA8  → I2C3_SCL
  PC9  → I2C3_SDA
  ```
- **Địa chỉ:** 0x3C (primary) hoặc 0x3D
- **Cấu hình:** 100 kHz, 7-bit addressing

**Code ví dụ trong dự án:**
```c
// File: Core/Src/ssd1306.c
#define SSD1306_I2C_ADDR 0x78  // 0x3C << 1

void ssd1306_WriteCommand(uint8_t command)
{
    uint8_t data[2] = {0x00, command};  // 0x00 = command mode
    HAL_I2C_Master_Transmit(&hi2c3, SSD1306_I2C_ADDR, data, 2, 100);
}

void SSD1306_UpdateScreen(void)
{
    // Update entire 128x64 display buffer
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_WriteCommand(0xB0 + page);  // Set page address
        ssd1306_WriteCommand(0x00);          // Set lower column
        ssd1306_WriteCommand(0x10);          // Set higher column
        
        HAL_I2C_Mem_Write(&hi2c3, SSD1306_I2C_ADDR, 0x40, 
                          I2C_MEMADD_SIZE_8BIT, 
                          &SSD1306_Buffer[page * 128], 128, 100);
    }
}
```

---

### 2.4. ADC1 (Analog-to-Digital Converter)

**Lý thuyết:**
- ADC chuyển đổi điện áp analog (0-3.3V) thành giá trị digital (0-4095 cho 12-bit)
- STM32F411 có ADC 12-bit với 16 kênh
- Công thức: `Voltage = (ADC_Value / 4095) × 3.3V`

**Trong dự án này:**
- **Thiết bị:** Soil Moisture Sensor
- **Chân kết nối:** PA4 → ADC1_IN4
- **Cấu hình:**
  - Resolution: 12-bit (0-4095)
  - Sampling time: 56 cycles
  - Conversion mode: Single

**Code ví dụ trong dự án:**
```c
// File: Core/Src/main.c
float ReadSoilMoisture(void)
{
    uint32_t adc_sum = 0;
    
    // Average 10 samples for stability
    for (int i = 0; i < 10; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    
    uint16_t adc_value = adc_sum / 10;
    
    // Convert to percentage (0% = dry, 100% = wet)
    // Assuming: 4095 = very dry (air), 0 = very wet (water)
    float moisture_percent = 100.0f - ((float)adc_value / 4095.0f * 100.0f);
    
    return moisture_percent;
}
```

**Calibration:**
- Đo giá trị ADC khi cảm biến trong không khí (dry): ~4095
- Đo giá trị ADC khi cảm biến trong nước (wet): ~0-500
- Map giá trị giữa 2 ngưỡng này thành phần trăm

---

### 2.5. UART2 (Universal Asynchronous Receiver Transmitter)

**Lý thuyết:**
- UART là giao thức nối tiếp không đồng bộ
- Không cần clock, sử dụng baudrate đã thỏa thuận
- Full-duplex (TX và RX đồng thời)

**Trong dự án này:**
- **Thiết bị kết nối:** Orange Pi 4A
- **Chân kết nối:**
  ```
  PA2 → USART2_TX  (Transmit)
  PA3 → USART2_RX  (Receive)
  ```
- **Cấu hình:**
  - Baudrate: 115200 bps
  - Word length: 8 bits
  - Stop bits: 1
  - Parity: None

**Code ví dụ trong dự án:**
```c
// File: Core/Src/main.c
void SendDataToUart(const char *data)
{
    // Acquire UART mutex
    if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)data, 
                         strlen(data), UART_TIMEOUT_MS);
        osMutexRelease(uartMutexHandle);
    }
}

// Example: Send JSON data
void UartCommunicationTask(void *argument)
{
    SensorData_t receivedData;
    
    for(;;)
    {
        if (osMessageQueueGet(sensorDataQueueHandle, 
                             &receivedData, NULL, 50) == osOK) 
        {
            char buffer[512];
            snprintf(buffer, sizeof(buffer),
                    "{\"temp\":%.2f,\"humi\":%.2f,\"pres\":%.2f,"
                    "\"co2\":%d,\"tvoc\":%d,\"lux\":%.2f,\"soil\":%.1f}\r\n",
                    receivedData.temperature,
                    receivedData.humidity,
                    receivedData.pressure,
                    receivedData.co2,
                    receivedData.tvoc,
                    receivedData.light_lux,
                    receivedData.soil_moisture);
            
            SendDataToUart(buffer);
        }
        
        osDelay(50);
    }
}
```

**Định dạng dữ liệu:**
- **JSON** (mặc định): Dễ parse, human-readable
- **CSV**: Compact, dễ import vào Excel
- **Plain text**: Đơn giản nhất

---

### 2.6. GPIO (General Purpose Input/Output)

**Lý thuyết:**
- GPIO là chân đa năng có thể cấu hình input hoặc output
- Output modes: Push-pull, Open-drain
- Input modes: Floating, Pull-up, Pull-down

**Trong dự án này:**
- **LED PC13:** Báo trạng thái hệ thống
  - Mode: Output Push-Pull
  - Speed: Low
  - Active: LOW (0V = LED ON, 3.3V = LED OFF)

**Code ví dụ trong dự án:**
```c
// File: Core/Src/main.c
void LedStatusTask(void *argument)
{
    for(;;)
    {
        bool anySensorConnected = systemStatus.bme280_connected || 
                                 systemStatus.ccs811_connected;
        
        if (anySensorConnected) {
            // LED ON (Active LOW)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        } else {
            // LED Blink
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
        
        osDelay(500);
    }
}
```

---

## 3. FREERTOS - TASKS VÀ CƠ CHẾ HOẠT ĐỘNG

### 3.1. Khái niệm Task trong FreeRTOS

**Task** là một thread/luồng thực thi độc lập. Mỗi task:
- Có **stack riêng** (lưu local variables, return addresses)
- Có **priority riêng** (0 = thấp nhất, 55 = cao nhất)
- Chạy như một **vòng lặp vô hạn** `for(;;)`
- Được **scheduler** quản lý và chuyển đổi

**Task States (Trạng thái):**
```
┌──────────┐  osDelay()     ┌──────────┐
│ Running  │───────────────▶│ Blocked  │
│          │                 │ (Waiting)│
└──────────┘                 └──────────┘
     ▲                            │
     │                            │ Timeout/Event
     │                            ▼
     │         Preempted      ┌──────────┐
     └────────────────────────│  Ready   │
        Higher Priority       └──────────┘
```

- **Running:** Task đang chạy trên CPU
- **Ready:** Task sẵn sàng chạy, chờ scheduler
- **Blocked:** Task đang chờ (delay, queue, mutex)

---

### 3.2. Cấu hình FreeRTOS

**File:** `Core/Inc/FreeRTOSConfig.h`

```c
// Tần số System Tick
#define configTICK_RATE_HZ                1000
// => 1000 Hz = 1 tick mỗi 1 ms

// Heap size - Bộ nhớ dành cho RTOS
#define configTOTAL_HEAP_SIZE             15360
// => 15 KB RAM cho tasks, queues, mutexes

// Số mức priority tối đa
#define configMAX_PRIORITIES              56
// => Priority từ 0 đến 55

// Cho phép preemption (chiếm quyền)
#define configUSE_PREEMPTION              1

// Kích hoạt Mutexes
#define configUSE_MUTEXES                 1

// Support for newlib (thread-safe malloc, printf...)
#define configUSE_NEWLIB_REENTRANT        1
```

**Tính toán Heap cần thiết:**
```
Tasks:
- SensorTask:   512 words × 4 = 2048 bytes
- UartTask:     768 words × 4 = 3072 bytes  
- LedTask:      256 words × 4 = 1024 bytes
- MonitorTask:  512 words × 4 = 2048 bytes
- OledTask:     512 words × 4 = 2048 bytes
Tổng Tasks: ~10.7 KB

Queues: ~3 KB
Mutexes: ~300 bytes
Overhead: ~500 bytes

Tổng: ~14.4 KB → Cấu hình 15 KB
```

---

### 3.3. Các Tasks trong dự án

#### **Task 1: SensorReadingTask**

**Thông số:**
```c
Priority: osPriorityHigh (cao nhất)
Stack: 512 words (2048 bytes)
Period: 3000 ms
```

**Chức năng:**
1. Đọc dữ liệu từ 4 sensors (BME280, CCS811, BH1750, Soil Moisture)
2. Sử dụng SPI Mutex để bảo vệ bus
3. Đưa dữ liệu vào sensorDataQueue
4. Kiểm tra cảnh báo tưới nước
5. Cập nhật LED status

**Code trong dự án:**
```c
// File: Core/Src/main.c (line 1434)
void SensorReadingTask(void *argument)
{
  SensorData_t sensorData;
  
  // Khởi tạo sensors một lần
  InitializeSensors();
  
  // Vòng lặp vô hạn
  for(;;)
  {
    // Acquire SPI mutex (timeout 1000ms)
    if (osMutexAcquire(spiMutexHandle, 1000) == osOK) {
      ReadAllSensorData(&sensorData);
      osMutexRelease(spiMutexHandle);
      
      // Cập nhật LED status
      bool sensors_ok = (systemStatus.bme280_connected || 
                        systemStatus.ccs811_connected);
      LED_SetSensorStatus(sensors_ok);
      
      // Gửi data vào queue (timeout 100ms)
      osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
    }
    
    // Delay 3 giây trước lần đọc tiếp theo
    osDelay(SENSOR_READ_PERIOD_MS);  // 3000 ms
  }
}
```

**Tại sao priority cao?**
- Sensors cần được đọc đúng lúc (real-time)
- Không được miss timing để đảm bảo độ chính xác

---

#### **Task 2: UartCommunicationTask**

**Thông số:**
```c
Priority: osPriorityAboveNormal
Stack: 768 words (3072 bytes)
Period: Non-blocking (50 ms check)
```

**Chức năng:**
1. Nhận data từ sensorDataQueue
2. Format data (JSON/CSV/Plain text)
3. Gửi qua UART đến Orange Pi
4. Sử dụng UART Mutex để tránh conflict

**Code trong dự án:**
```c
// File: Core/Src/main.c (line 1512)
void UartCommunicationTask(void *argument)
{
  SensorData_t receivedData;
  
  for(;;)
  {
    // Lấy data từ queue (timeout 50ms)
    if (osMessageQueueGet(sensorDataQueueHandle, 
                         &receivedData, NULL, 50) == osOK) 
    {
      // Acquire UART mutex
      if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
        // Format data thành JSON
        char buffer[512];
        snprintf(buffer, sizeof(buffer),
                "{\"temp\":%.2f,\"humi\":%.2f,\"pres\":%.2f,"
                "\"co2\":%d,\"tvoc\":%d}\r\n",
                receivedData.temperature,
                receivedData.humidity,
                receivedData.pressure,
                receivedData.co2,
                receivedData.tvoc);
        
        // Send via UART
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 
                         strlen(buffer), 100);
        osMutexRelease(uartMutexHandle);
      }
    }
    
    osDelay(50);
  }
}
```

**Tại sao stack lớn (3072 bytes)?**
- Cần buffer lớn để format JSON string (512 bytes)
- `snprintf()` tốn nhiều stack

---

#### **Task 3: LedStatusTask**

**Thông số:**
```c
Priority: osPriorityBelowNormal (thấp)
Stack: 256 words (1024 bytes)
Period: 500 ms
```

**Chức năng:**
- Kiểm tra trạng thái sensors
- Nhấp nháy LED nếu có lỗi
- Bật LED liên tục nếu OK

**Code trong dự án:**
```c
// File: Core/Src/main.c (line 1563)
void LedStatusTask(void *argument)
{
  for(;;)
  {
    bool anySensorConnected = systemStatus.bme280_connected || 
                             systemStatus.ccs811_connected;
    
    if (anySensorConnected) {
      // LED ON (Active LOW)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {
      // LED Blink
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    
    osDelay(LED_BLINK_PERIOD_MS);  // 500 ms
  }
}
```

**Tại sao priority thấp?**
- LED chỉ là visual feedback, không quan trọng
- Có thể bị chiếm quyền bởi tasks quan trọng hơn

---

#### **Task 4: SystemMonitorTask**

**Thông số:**
```c
Priority: osPriorityLow
Stack: 512 words
Period: 60000 ms (1 phút)
```

**Chức năng:**
- Kiểm tra kết nối sensors
- Cập nhật uptime
- Gửi heartbeat message

**Code trong dự án:**
```c
// File: Core/Src/main.c (line 1593)
void SystemMonitorTask(void *argument)
{
  for(;;)
  {
    UpdateSystemStatus();
    
    // Gửi heartbeat
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "{\"type\":\"HEARTBEAT\",\"uptime\":%lu,"
             "\"bme280\":%s,\"ccs811\":%s}\r\n",
             systemStatus.uptime_seconds,
             systemStatus.bme280_connected ? "true" : "false",
             systemStatus.ccs811_connected ? "true" : "false");
    
    UartMessage_t msg;
    strcpy(msg.data, buffer);
    msg.length = strlen(buffer);
    osMessageQueuePut(uartTxQueueHandle, &msg, 0, 100);
    
    osDelay(MONITOR_PERIOD_MS);  // 60000 ms
  }
}
```

---

#### **Task 5: OledDisplayTask**

**Thông số:**
```c
Priority: osPriorityNormal
Stack: 512 words
```

**Chức năng:**
- Hiển thị data lên màn hình OLED
- Tự động chuyển trang
- Sử dụng OLED Mutex

**Code trong dự án:**
```c
// File: Core/Src/main.c (line 1623)
void OledDisplayTask(void *argument)
{
  for(;;)
  {
    if (systemStatus.oled_connected) {
      // Acquire OLED mutex
      if (osMutexAcquire(oledMutexHandle, 100) == osOK) {
        DisplaySensorDataOnOled(&lastSensorData);
        osMutexRelease(oledMutexHandle);
      }
      osDelay(5000);
    } else {
      // Try to reinitialize OLED
      if (SSD1306_Init() == 1) {
        systemStatus.oled_connected = true;
      }
      osDelay(2000);
    }
  }
}
```

---

### 3.4. Scheduler và Context Switching

**FreeRTOS Scheduler:**
- **Preemptive priority-based:** Task ưu tiên cao có thể chiếm quyền
- **Time-slicing:** Tasks cùng priority chia sẻ CPU theo round-robin
- **SysTick interrupt:** Mỗi 1 ms (1000 Hz)

**Context Switching:**
```
Task A (Priority High)     Scheduler     Task B (Priority Low)
      │                        │                    │
      ├────── Running ─────────┤                    │
      │                        │                    │
      ├─── osDelay(1000) ─────▶│                    │
      │                        ├─── Switch to B ───▶│
      │    (Blocked)           │                    │
      │                        │                    │
      │                        │◄─── Task B done ───┤
      │◄─── Wake up ───────────┤                    │
      │                        │                    │
```

**Context switch process:**
1. Lưu context của Task hiện tại (registers, PC, SP)
2. Chọn Task tiếp theo (highest priority ready)
3. Khôi phục context của Task mới
4. Tiếp tục thực thi Task mới

---

## 4. FREERTOS - QUEUE, SEMAPHORE, MUTEX

### 4.1. Queue (Hàng đợi)

**Khái niệm:**
- Queue là cấu trúc dữ liệu **FIFO** (First-In-First-Out)
- Thread-safe communication giữa các tasks
- Không cần mutex riêng

**Trong dự án:**

#### **Queue 1: sensorDataQueue**
```c
Type: SensorData_t (64 bytes/message)
Capacity: 5 messages
Total Size: 5 × 64 = 320 bytes
```

**Cấu trúc dữ liệu:**
```c
// File: Core/Src/main.c (line 40)
typedef struct {
    float temperature;
    float humidity; 
    float pressure;
    uint16_t co2;
    uint16_t tvoc;
    float light_lux;
    float soil_moisture;
    uint32_t timestamp;
    uint8_t bme_valid : 1;
    uint8_t ccs_valid : 1;
    uint8_t bh1750_valid : 1;
    uint8_t soil_valid : 1;
} SensorData_t;
```

**Tạo Queue:**
```c
// File: Core/Src/main.c (line 349)
sensorDataQueueHandle = osMessageQueueNew(
    SENSOR_QUEUE_SIZE,      // 5 messages
    sizeof(SensorData_t),   // 64 bytes per message
    NULL                    // Default attributes
);

if (sensorDataQueueHandle == NULL) {
    Error_Handler();
}
```

**Producer (SensorTask) - Gửi data:**
```c
SensorData_t sensorData;
ReadAllSensorData(&sensorData);

// Put vào queue (timeout 100ms)
osStatus_t status = osMessageQueuePut(
    sensorDataQueueHandle,
    &sensorData,
    0,      // Priority (0 = normal)
    100     // Timeout (100 ticks = 100 ms)
);

if (status != osOK) {
    systemStatus.sensor_errors++;
}
```

**Consumer (UartTask) - Lấy data:**
```c
SensorData_t receivedData;

// Get từ queue (timeout 50ms)
osStatus_t status = osMessageQueueGet(
    sensorDataQueueHandle,
    &receivedData,
    NULL,   // Priority output (không quan tâm)
    50      // Timeout (50 ms)
);

if (status == osOK) {
    // Process data
    FormatSensorData(&receivedData, buffer, sizeof(buffer));
}
```

**Tại sao capacity = 5?**
- SensorTask gửi mỗi 3 giây
- UartTask nhận và xử lý nhanh (~100 ms)
- 5 messages = buffer 15 giây data → đủ dự phòng

---

#### **Queue 2: uartTxQueue**
```c
Type: UartMessage_t (258 bytes/message)
Capacity: 10 messages
Total Size: 10 × 258 = 2580 bytes
```

**Cấu trúc:**
```c
typedef struct {
    char data[256];
    uint16_t length;
} UartMessage_t;
```

**Luồng dữ liệu:**
```
SensorTask ──▶ sensorDataQueue ──▶ UartTask ──▶ UART2
MonitorTask ──▶ uartTxQueue ──────▶ UartTask ──▶ UART2
```

---

### 4.2. Mutex (Khóa tương hỗ)

**Khái niệm:**
- Mutex = **Mutual Exclusion** (loại trừ lẫn nhau)
- Bảo vệ tài nguyên chia sẻ (shared resources)
- Chỉ 1 task có thể acquire mutex tại một thời điểm

**Khi nào cần Mutex?**
- Nhiều tasks truy cập cùng hardware (SPI, UART, I2C)
- Nhiều tasks đọc/ghi cùng biến global
- Tránh race condition

**Priority Inheritance:**
- Nếu Task A (low priority) đang giữ mutex
- Task B (high priority) muốn acquire mutex
- Task A tạm thời được nâng priority = Task B
- Tránh priority inversion

---

**Trong dự án:**

#### **Mutex 1: spiMutexHandle**
```c
Purpose: Bảo vệ SPI3 bus (BME280)
```

**Tạo Mutex:**
```c
// File: Core/Src/main.c (line 320)
spiMutexHandle = osMutexNew(NULL);  // Default attributes
if (spiMutexHandle == NULL) {
    Error_Handler();
}
```

**Sử dụng:**
```c
// File: Core/Src/main.c (SensorReadingTask)
// Acquire mutex trước khi truy cập SPI
if (osMutexAcquire(spiMutexHandle, 1000) == osOK) {
    // Critical section - chỉ 1 task được vào
    ReadAllSensorData(&sensorData);
    
    // Release mutex sau khi xong
    osMutexRelease(spiMutexHandle);
} else {
    // Timeout - không lấy được mutex
    systemStatus.sensor_errors++;
}
```

---

#### **Mutex 2: uartMutexHandle**
```c
Purpose: Bảo vệ UART2 (tránh conflict khi nhiều tasks gửi data)
```

**Sử dụng:**
```c
// Acquire UART mutex
if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 
                     strlen(buffer), 100);
    osMutexRelease(uartMutexHandle);
} else {
    systemStatus.uart_errors++;
}
```

---

#### **Mutex 3: oledMutexHandle**
```c
Purpose: Bảo vệ I2C3 bus (OLED display)
```

**Sử dụng:**
```c
if (osMutexAcquire(oledMutexHandle, 100) == osOK) {
    DisplaySensorDataOnOled(&sensorData);
    osMutexRelease(oledMutexHandle);
}
```

---

### 4.3. Semaphore (trong dự án này không dùng)

**Binary Semaphore:**
- Dùng để đồng bộ hóa tasks (signaling)
- Giống mutex nhưng không có priority inheritance
- Ví dụ: Task A báo cho Task B bắt đầu công việc

**Counting Semaphore:**
- Đếm số lượng tài nguyên khả dụng
- Ví dụ: Có 3 buffer available, cho phép 3 tasks truy cập đồng thời

---

## 5. CODE TRONG DỰ ÁN

### 5.1. Khởi tạo RTOS trong main()

**File:** `Core/Src/main.c`

```c
int main(void)
{
  // 1. HAL Initialization
  HAL_Init();
  SystemClock_Config();
  
  // 2. Peripheral Initialization
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  
  // 3. Initialize RTOS Kernel
  osKernelInitialize();
  
  // 4. Create Mutexes
  spiMutexHandle = osMutexNew(NULL);
  uartMutexHandle = osMutexNew(NULL);
  oledMutexHandle = osMutexNew(NULL);
  
  // 5. Create Queues
  sensorDataQueueHandle = osMessageQueueNew(
      SENSOR_QUEUE_SIZE, 
      sizeof(SensorData_t), 
      NULL);
  
  uartTxQueueHandle = osMessageQueueNew(
      UART_QUEUE_SIZE, 
      sizeof(UartMessage_t), 
      NULL);
  
  // 6. Create Tasks
  sensorTaskHandle = osThreadNew(
      SensorReadingTask, 
      NULL, 
      &sensorTask_attributes);
  
  uartTaskHandle = osThreadNew(
      UartCommunicationTask, 
      NULL, 
      &uartTask_attributes);
  
  ledTaskHandle = osThreadNew(
      LedStatusTask, 
      NULL, 
      &ledTask_attributes);
  
  monitorTaskHandle = osThreadNew(
      SystemMonitorTask, 
      NULL, 
      &monitorTask_attributes);
  
  oledTaskHandle = osThreadNew(
      OledDisplayTask, 
      NULL, 
      &oledTask_attributes);
  
  // 7. Start Scheduler (never returns)
  osKernelStart();
  
  // Never reached
  while (1);
}
```

---

### 5.2. Task Attributes

```c
// File: Core/Src/main.c (line 185)
const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = SENSOR_TASK_STACK_SIZE * 4,  // 512 words = 2048 bytes
  .priority = SENSOR_TASK_PRIORITY,           // osPriorityHigh
};

const osThreadAttr_t uartTask_attributes = {
  .name = "UartTask", 
  .stack_size = UART_TASK_STACK_SIZE * 4,    // 768 words = 3072 bytes
  .priority = UART_TASK_PRIORITY,             // osPriorityAboveNormal
};

const osThreadAttr_t ledTask_attributes = {
  .name = "LedTask",
  .stack_size = LED_TASK_STACK_SIZE * 4,     // 256 words = 1024 bytes
  .priority = LED_TASK_PRIORITY,              // osPriorityBelowNormal
};
```

---

### 5.3. System Status Structure

```c
// File: Core/Src/main.c (line 65)
typedef struct {
    bool bme280_connected;
    bool ccs811_connected;
    bool bh1750_connected;
    bool soil_sensor_active;
    bool oled_connected;
    uint32_t sensor_errors;
    uint32_t uart_errors;
    uint32_t uptime_seconds;
} SystemStatus_t;

// Global instance
SystemStatus_t systemStatus = {false, false, false, false, false, 0, 0, 0};
```

---

### 5.4. Format Sensor Data (JSON)

```c
void FormatSensorData(SensorData_t *data, char *buffer, size_t size)
{
    snprintf(buffer, size,
             "{\"device\":\"STM32F411\","
             "\"timestamp\":%lu,"
             "\"temp\":%.2f,"
             "\"humi\":%.2f,"
             "\"pres\":%.2f,"
             "\"co2\":%d,"
             "\"tvoc\":%d,"
             "\"lux\":%.2f,"
             "\"soil\":%.1f,"
             "\"bme\":\"%s\","
             "\"ccs\":\"%s\"}\r\n",
             data->timestamp,
             data->temperature,
             data->humidity,
             data->pressure,
             data->co2,
             data->tvoc,
             data->light_lux,
             data->soil_moisture,
             data->bme_valid ? "OK" : "ERR",
             data->ccs_valid ? "OK" : "ERR");
}
```

---

### 5.5. Irrigation Check

```c
void CheckIrrigationNeeds(SensorData_t *data)
{
    if (data->soil_valid && 
        data->soil_moisture < SOIL_MOISTURE_THRESHOLD)  // 30%
    {
        // Send irrigation alert
        UartMessage_t alert;
        snprintf(alert.data, sizeof(alert.data),
                 "{\"type\":\"ALERT\",\"message\":\"Irrigation needed\","
                 "\"soil\":%.1f%%}\r\n",
                 data->soil_moisture);
        alert.length = strlen(alert.data);
        
        osMessageQueuePut(uartTxQueueHandle, &alert, 0, 100);
    }
}
```

---

## 6. CÂU HỎI VIVA THƯỜNG GẶP

### 6.1. Câu hỏi về hệ thống

**Q1: Giới thiệu sơ lược về dự án của bạn?**
> **A:** Dự án là hệ thống giám sát môi trường sử dụng STM32F411CEU6 với FreeRTOS. Hệ thống đọc dữ liệu từ 4 loại cảm biến (nhiệt độ/độ ẩm/áp suất từ BME280, CO2/TVOC từ CCS811, ánh sáng từ BH1750, độ ẩm đất từ ADC), hiển thị lên OLED và truyền qua UART đến Orange Pi 4A. Sử dụng FreeRTOS để quản lý 5 tasks chạy song song với các mức ưu tiên khác nhau.

**Q2: Tại sao lại sử dụng RTOS thay vì bare-metal?**
> **A:** RTOS giúp:
> - Quản lý đa nhiệm dễ dàng (5 tasks chạy đồng thời)
> - Ưu tiên công việc quan trọng (sensor reading > LED blinking)
> - Code dễ bảo trì, mở rộng
> - Tránh polling loop phức tạp
> - Thread-safe communication qua Queue và Mutex

**Q3: Phần cứng gồm những gì?**
> **A:** 
> - Vi điều khiển: STM32F411CEU6 (100MHz, 512KB Flash, 128KB RAM)
> - Cảm biến: BME280 (SPI3), CCS811 (I2C2), BH1750 (I2C2), Soil sensor (ADC1)
> - Display: SSD1306 OLED 128x64 (I2C3)
> - Communication: UART2 đến Orange Pi 4A (115200 baud)
> - Indicator: LED PC13

---

### 6.2. Câu hỏi về Tasks

**Q4: Có bao nhiêu tasks? Ưu tiên như thế nào?**
> **A:** Có 5 tasks chính:
> 1. **SensorReadingTask** (High) - Đọc sensors, quan trọng nhất
> 2. **UartCommunicationTask** (AboveNormal) - Gửi data đi
> 3. **OledDisplayTask** (Normal) - Hiển thị OLED
> 4. **LedStatusTask** (BelowNormal) - Nhấp nháy LED
> 5. **SystemMonitorTask** (Low) - Heartbeat, ít quan trọng nhất

**Q5: SensorReadingTask làm gì?**
> **A:** 
> - Đọc dữ liệu từ BME280 (SPI3), CCS811/BH1750 (I2C2), Soil (ADC1)
> - Acquire SPI mutex trước khi đọc
> - Đưa data vào sensorDataQueue
> - Kiểm tra cảnh báo tưới nước
> - Cập nhật LED status
> - Chạy mỗi 3 giây (osDelay(3000))

**Q6: Tại sao SensorTask có priority cao nhất?**
> **A:** Vì sensors cần được đọc đúng thời điểm để đảm bảo độ chính xác real-time. Nếu bị tasks khác chiếm quyền lâu, có thể miss data hoặc đọc data cũ.

**Q7: UartCommunicationTask hoạt động ra sao?**
> **A:**
> - Lấy data từ sensorDataQueue (osMessageQueueGet)
> - Format thành JSON string
> - Acquire UART mutex
> - Gửi qua UART2 đến Orange Pi
> - Release mutex
> - Non-blocking, check queue mỗi 50ms

---

### 6.3. Câu hỏi về Queues

**Q8: Có bao nhiêu queues? Dùng làm gì?**
> **A:** Có 2 queues:
> 1. **sensorDataQueue** (5 messages × 64 bytes): Truyền sensor data từ SensorTask đến UartTask
> 2. **uartTxQueue** (10 messages × 258 bytes): Truyền system messages (heartbeat, alerts)

**Q9: Queue hoạt động như thế nào?**
> **A:** 
> - Queue là FIFO (First-In-First-Out), thread-safe
> - Producer (SensorTask) gọi `osMessageQueuePut()` để gửi data
> - Consumer (UartTask) gọi `osMessageQueueGet()` để nhận data
> - Nếu queue đầy, Put sẽ block hoặc timeout
> - Nếu queue rỗng, Get sẽ block hoặc timeout

**Q10: Tại sao sensorDataQueue capacity = 5?**
> **A:** 
> - SensorTask gửi mỗi 3 giây
> - UartTask xử lý nhanh (~100ms)
> - 5 messages = buffer 15 giây data
> - Đủ dự phòng nếu UartTask bị delay tạm thời

---

### 6.4. Câu hỏi về Mutexes

**Q11: Mutex là gì? Dùng khi nào?**
> **A:** 
> - Mutex = Mutual Exclusion (khóa tương hỗ)
> - Bảo vệ tài nguyên chia sẻ (SPI, UART, I2C)
> - Chỉ 1 task được acquire mutex tại một thời điểm
> - Tránh race condition khi nhiều tasks truy cập cùng hardware

**Q12: Có bao nhiêu mutexes trong dự án?**
> **A:** Có 3 mutexes:
> 1. **spiMutexHandle** - Bảo vệ SPI3 bus (BME280)
> 2. **uartMutexHandle** - Bảo vệ UART2 (tránh conflict khi gửi data)
> 3. **oledMutexHandle** - Bảo vệ I2C3 bus (OLED display)

**Q13: Giải thích cách dùng Mutex trong SensorTask?**
> **A:**
> ```c
> // Acquire mutex (timeout 1000ms)
> if (osMutexAcquire(spiMutexHandle, 1000) == osOK) {
>     // Critical section - chỉ 1 task vào được
>     ReadAllSensorData(&sensorData);
>     
>     // Release mutex ngay sau khi xong
>     osMutexRelease(spiMutexHandle);
> } else {
>     // Timeout - không lấy được mutex
>     systemStatus.sensor_errors++;
> }
> ```

**Q14: Priority Inversion là gì?**
> **A:** 
> - Task A (low priority) đang giữ mutex
> - Task C (high priority) muốn acquire mutex → bị block
> - Task B (medium priority) chạy và chiếm quyền Task A
> - Task C bị chờ lâu dù có priority cao
> - **Giải pháp:** FreeRTOS tự động nâng priority của Task A = Task C (priority inheritance)

---

### 6.5. Câu hỏi về giao tiếp ngoại vi

**Q15: Có những giao tiếp nào trong dự án?**
> **A:** 
> - **SPI3:** BME280 (nhiệt độ, độ ẩm, áp suất)
> - **I2C2:** CCS811 (CO2, TVOC), BH1750 (ánh sáng)
> - **I2C3:** SSD1306 OLED
> - **ADC1:** Soil Moisture Sensor
> - **UART2:** Communication với Orange Pi 4A
> - **GPIO:** LED PC13 (Active LOW)

**Q16: Tại sao BME280 dùng SPI còn CCS811 dùng I2C?**
> **A:** 
> - BME280 hỗ trợ cả SPI và I2C, chọn SPI vì:
>   - Tốc độ cao hơn (MHz vs 100kHz)
>   - Không xung đột địa chỉ với CCS811
> - CCS811 chỉ hỗ trợ I2C nên phải dùng I2C2

**Q17: Tại sao OLED dùng bus I2C riêng (I2C3)?**
> **A:** 
> - Tránh xung đột với cảm biến trên I2C2
> - OLED cập nhật thường xuyên, chiếm bus lâu
> - Tách riêng giúp cảm biến không bị ảnh hưởng

**Q18: UART gửi data định dạng gì?**
> **A:** Có 3 định dạng:
> - **JSON** (mặc định): Human-readable, dễ parse
>   ```json
>   {"temp":25.6,"humi":65.2,"pres":1013.2}
>   ```
> - **CSV**: Compact, dễ import Excel
>   ```
>   123456,25.6,65.2,1013.2
>   ```
> - **Plain text**: Đơn giản nhất
>   ```
>   TEMP:25.6 HUMI:65.2 PRES:1013.2
>   ```

---

### 6.6. Câu hỏi về cấu hình

**Q19: FreeRTOS dùng bao nhiêu RAM?**
> **A:** 
> - configTOTAL_HEAP_SIZE = 15 KB (15360 bytes)
> - Bao gồm: Tasks (~10.7 KB) + Queues (~3 KB) + Mutexes + Overhead

**Q20: System Tick là gì?**
> **A:** 
> - configTICK_RATE_HZ = 1000 Hz
> - Nghĩa là có 1000 interrupts/giây → 1 tick = 1 ms
> - osDelay(100) = delay 100 ticks = 100 ms

**Q21: Preemption là gì?**
> **A:** 
> - configUSE_PREEMPTION = 1
> - Task ưu tiên cao có thể chiếm quyền task ưu tiên thấp
> - Ví dụ: SensorTask (High) chiếm quyền từ LedTask (BelowNormal) khi ready

---

### 6.7. Câu hỏi code chi tiết

**Q22: Giải thích đoạn code đọc BME280 qua SPI?**
> **A:**
> ```c
> uint8_t BME280_ReadRegister(uint8_t reg)
> {
>     uint8_t txData[2], rxData[2];
>     txData[0] = reg | 0x80;  // Set bit 7 = read mode
>     
>     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // CS LOW
>     HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 2, HAL_MAX_DELAY);
>     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // CS HIGH
>     
>     return rxData[1];  // Byte thứ 2 là data
> }
> ```

**Q23: Giải thích code gửi data qua UART với Mutex?**
> **A:**
> ```c
> // Acquire UART mutex (timeout 1000ms)
> if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
>     // Critical section - chỉ 1 task gửi UART
>     HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 
>                      strlen(buffer), 100);
>     
>     // Release mutex ngay
>     osMutexRelease(uartMutexHandle);
> } else {
>     systemStatus.uart_errors++;  // Log error nếu timeout
> }
> ```

**Q24: Code đọc ADC (Soil Moisture) như thế nào?**
> **A:**
> ```c
> float ReadSoilMoisture(void)
> {
>     uint32_t adc_sum = 0;
>     
>     // Lấy trung bình 10 mẫu
>     for (int i = 0; i < 10; i++) {
>         HAL_ADC_Start(&hadc1);
>         HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
>         adc_sum += HAL_ADC_GetValue(&hadc1);
>         HAL_ADC_Stop(&hadc1);
>     }
>     
>     uint16_t adc_value = adc_sum / 10;
>     
>     // Convert sang % (4095 = khô, 0 = ướt)
>     float moisture_percent = 100.0f - ((float)adc_value / 4095.0f * 100.0f);
>     
>     return moisture_percent;
> }
> ```

---

### 6.8. Câu hỏi troubleshooting

**Q25: Làm sao biết task bị stack overflow?**
> **A:** 
> - FreeRTOS sẽ gọi `vApplicationStackOverflowHook()`
> - Có thể dùng `uxTaskGetStackHighWaterMark()` để check stack usage
> - Nếu stack overflow: tăng stack size trong task attributes

**Q26: Nếu sensor không hoạt động, hệ thống xử lý thế nào?**
> **A:** 
> - SensorTask cố gắng khởi tạo lại sensor
> - LED nhấp nháy báo lỗi
> - Đếm số lỗi trong `systemStatus.sensor_errors`
> - SystemMonitorTask gửi heartbeat báo sensor disconnected

**Q27: Mutex timeout bao nhiêu là hợp lý?**
> **A:** 
> - Timeout = thời gian tối đa task khác giữ mutex
> - SPI: 1000ms (đọc sensor mất ~100ms, dư 10x)
> - UART: 1000ms (gửi ~100 bytes @ 115200 baud ~ 10ms)
> - OLED: 100ms (cập nhật nhanh)

---

### 6.9. Tổng kết

**Các điểm quan trọng cần nhớ:**

1. **Hệ thống:** STM32F411 + FreeRTOS, 5 tasks, 2 queues, 3 mutexes
2. **Giao tiếp:** SPI3 (BME280), I2C2 (CCS811, BH1750), I2C3 (OLED), ADC1 (Soil), UART2 (Orange Pi)
3. **Tasks:** SensorTask (High) → Queue → UartTask (AboveNormal)
4. **Priority:** High > AboveNormal > Normal > BelowNormal > Low
5. **Queue:** FIFO, thread-safe, osMessageQueuePut/Get
6. **Mutex:** Bảo vệ shared resources, Acquire/Release, priority inheritance
7. **Config:** 15KB heap, 1ms tick, preemption enabled

---

**Chúc bạn bảo vệ tốt! 🚀**
