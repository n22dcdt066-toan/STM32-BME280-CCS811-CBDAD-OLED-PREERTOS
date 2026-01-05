# HỆ THỐNG GIÁM SÁT MÔI TRƯỜNG STM32F411CEU6 + FreeRTOS

**Nhóm thực hiện:** 8 
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

Hệ thống giám sát môi trường sử dụng STM32F411CEU6 (Black Pill) kết hợp với FreeRTOS để đọc dữ liệu từ nhiều cảm biến và truyền thông tin qua UART đến Orange Pi 4A. 

**Mục đích:**
- Giám sát liên tục các thông số môi trường (nhiệt độ, độ ẩm, áp suất, CO2, TVOC)
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

**Cảm biến:**
- **BME280**:  Cảm biến môi trường (Nhiệt độ, Độ ẩm, Áp suất) - Giao tiếp I2C
- **CCS811**: Cảm biến chất lượng không khí (eCO2, TVOC) - Giao tiếp I2C

**Thiết bị ngoại vi:**
- **LED PC13**: Báo trạng thái hệ thống (Active LOW)
- **Orange Pi 4A**: Nhận dữ liệu qua UART2

### 1.3. Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────────────┐
│                   STM32F411CEU6 + FreeRTOS                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              FreeRTOS Scheduler                     │   │
│  │  (Preemptive Priority-based Multitasking)          │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│         ┌────────────────┼────────────────┐                │
│         │                │                │                │
│    ┌────▼─────┐    ┌────▼─────┐    ┌────▼─────┐          │
│    │ Sensor   │    │   UART   │    │   LED    │          │
│    │   Task   │    │   Task   │    │   Task   │          │
│    │(Priority │    │(Priority │    │(Priority │          │
│    │  High)   │    │ AbvNorm) │    │  BelNorm)│          │
│    └────┬─────┘    └────▲─────┘    └──────────┘          │
│         │               │                                  │
│         │  ┌──────────┐ │                                  │
│         └─►│  Queue   │─┘                                  │
│            │(Sensor   │                                    │
│            │ Data)    │                                    │
│            └──────────┘                                    │
│                                                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                │
│  │ I2C Mutex│  │UART Mutex│  │SystemMon │                │
│  │(BME280,  │  │ (UART2)  │  │  Task    │                │
│  │ CCS811)  │  │          │  │(Priority │                │
│  └──────────┘  └──────────┘  │   Low)   │                │
│                               └──────────┘                │
└─────────────────────────────────────────────────────────────┘
         │                              │
    ┌────▼─────┐                  ┌────▼──────┐
    │  I2C2    │                  │  UART2    │
    │  Bus     │                  │ (115200)  │
    └────┬─────┘                  └────┬──────┘
         │                              │
    ┌────▼─────┐                  ┌────▼──────┐
    │ BME280   │                  │Orange Pi  │
    │ CCS811   │                  │   4A      │
    └──────────┘                  └───────────┘
```

### 1.4. Nguyên lý hoạt động

1. **SensorReadingTask** đọc dữ liệu từ BME280 và CCS811 qua I2C2 (với mutex protection)
2. Dữ liệu được đưa vào **Queue** (sensorDataQueue)
3. **UartCommunicationTask** lấy dữ liệu từ Queue, format thành JSON/CSV/Plain text
4. Dữ liệu được gửi qua UART2 đến Orange Pi 4A (với mutex protection)
5. **LedStatusTask** cập nhật trạng thái LED dựa trên tình trạng cảm biến
6. **SystemMonitorTask** gửi heartbeat message định kỳ và giám sát hệ thống

---

## 2. CÁC GIAO TIẾP NGOẠI VI

### 2.1. I2C2 (Inter-Integrated Circuit)

**Mục đích:** Giao tiếp với cảm biến BME280 và CCS811

#### Cấu hình phần cứng
```c
// Pin Configuration
PB3  :  I2C2_SDA (Serial Data Line)
PB10 : I2C2_SCL (Serial Clock Line)

// I2C Settings
Mode       : I2C Master
Speed      : 100 kHz (Standard Mode)
Addressing : 7-bit address mode
```

#### Địa chỉ I2C
```c
BME280 :  0x76 << 1 = 0xEC (Write), 0xED (Read)
CCS811 : 0x5A << 1 = 0xB4 (Write), 0xB5 (Read)
```

#### Code ví dụ - Đọc BME280
```c
// Trong SensorReadingTask
void SensorReadingTask(void *argument)
{
  SensorData_t sensorData;
  BME280_Init(&hi2c2);  // Khởi tạo BME280
  CCS811_Init(&hi2c2);  // Khởi tạo CCS811
  
  for(;;)
  {
    // Acquire I2C mutex (timeout 1000ms)
    if (osMutexAcquire(i2cMutexHandle, 1000) == osOK) 
    {
      // Critical Section - Chỉ 1 task truy cập I2C
      
      // Đọc BME280
      if (BME280_ReadAll(&hi2c2, &sensorData. temperature, 
                         &sensorData.humidity, &sensorData.pressure) == HAL_OK) 
      {
        sensorData.bme280_status = 1; // OK
      } else {
        sensorData.bme280_status = 0; // Error
      }
      
      // Đọc CCS811
      if (CCS811_ReadData(&hi2c2, &sensorData.co2, &sensorData.tvoc) == HAL_OK) 
      {
        sensorData.ccs811_status = 1; // OK
        
        // Environmental compensation cho CCS811
        CCS811_SetEnvironmentalData(sensorData.temperature, 
                                    sensorData.humidity);
      } else {
        sensorData.ccs811_status = 0; // Error
      }
      
      // Release mutex
      osMutexRelease(i2cMutexHandle);
      
      // Gửi dữ liệu vào queue
      osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
    }
    
    // Delay 3 giây
    osDelay(3000);
  }
}
```

#### Lý thuyết I2C

**Đặc điểm:**
- Giao thức đồng bộ (synchronous)
- 2 dây:  SDA (data), SCL (clock)
- Multi-master, multi-slave
- Tốc độ: 100 kHz (Standard), 400 kHz (Fast), 3. 4 MHz (High-speed)

**Cơ chế hoạt động:**
1. START condition: SDA HIGH → LOW khi SCL HIGH
2. Gửi địa chỉ slave (7-bit) + R/W bit
3. Slave gửi ACK
4. Truyền dữ liệu (8-bit) + ACK/NACK
5. STOP condition: SDA LOW → HIGH khi SCL HIGH

**Ưu điểm:**
- Chỉ cần 2 dây
- Hỗ trợ nhiều slave
- Có cơ chế ACK/NACK

**Nhược điểm:**
- Tốc độ thấp hơn SPI
- Cần pull-up resistors (4.7kΩ)
- Phức tạp hơn UART

---

### 2.2. UART2 (Universal Asynchronous Receiver-Transmitter)

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
// Trong UartCommunicationTask
void UartCommunicationTask(void *argument)
{
  SensorData_t sensorData;
  char uartBuffer[256];
  
  for(;;)
  {
    // Chờ nhận dữ liệu từ queue (timeout 1000ms)
    if (osMessageQueueGet(sensorDataQueueHandle, &sensorData, NULL, 1000) == osOK)
    {
      // Format dữ liệu thành JSON
      int len = snprintf(uartBuffer, sizeof(uartBuffer),
        "{\"device\": \"STM32F411_RTOS\","
        "\"temp\": %.2f,"
        "\"humi\":%.2f,"
        "\"pres\":%.2f,"
        "\"co2\":%d,"
        "\"tvoc\":%d,"
        "\"bme\":\"%s\","
        "\"ccs\":\"%s\"}\r\n",
        sensorData.temperature,
        sensorData.humidity,
        sensorData.pressure,
        sensorData.co2,
        sensorData.tvoc,
        sensorData. bme280_status ? "OK" : "ERROR",
        sensorData.ccs811_status ? "OK" :  "ERROR"
      );
      
      // Acquire UART mutex (timeout 1000ms)
      if (osMutexAcquire(uartMutexHandle, 1000) == osOK)
      {
        // Truyền qua UART (timeout 100ms)
        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, len, 100);
        
        // Release mutex
        osMutexRelease(uartMutexHandle);
      }
    }
  }
}
```

#### Lý thuyết UART

**Đặc điểm:**
- Giao thức bất đồng bộ (asynchronous) - không cần clock chung
- 2 dây: TX, RX (+ GND)
- Full-duplex (truyền và nhận đồng thời)
- Point-to-point (1-1)

**Cơ chế hoạt động:**
1. Idle state: Line ở mức HIGH
2. Start bit: 1 bit LOW
3. Data bits: 5-9 bits (thường 8 bits)
4. Parity bit: Optional (kiểm tra lỗi)
5. Stop bit: 1-2 bits HIGH

**Frame format (8N1):**
```
┌─────┬───┬───┬───┬───┬───┬───┬───┬───┬──────┐
│START│ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │STOP  │
│  0  │   │   │   DATA BITS   │   │   │  1   │
└─────┴───┴───┴───┴───┴───┴───┴───┴───┴──────┘
```

**Tính baud rate:**
```c
Baud Rate = fPCLK / (16 × USARTDIV)
// VD: 115200 = 100MHz / (16 × 54. 25)
```

**Ưu điểm:**
- Đơn giản, dễ sử dụng
- Khoảng cách xa (với RS232/RS485)
- Không cần clock chung

**Nhược điểm:**
- Chỉ kết nối 1-1
- Tốc độ giới hạn
- Cần thống nhất baud rate 2 bên

---

### 2.3. GPIO (General Purpose Input/Output)

**LED PC13:** Báo trạng thái hệ thống

#### Cấu hình
```c
// Pin: PC13
// Mode: Output Push-Pull
// Pull:  No pull-up, no pull-down
// Speed: Low
// Active: LOW (LED sáng khi PC13 = 0)
```

#### Code ví dụ - Điều khiển LED
```c
// Trong LedStatusTask
void LedStatusTask(void *argument)
{
  for(;;)
  {
    // Kiểm tra trạng thái cảm biến
    if (systemStatus.bme280_ok || systemStatus.ccs811_ok) 
    {
      // Ít nhất 1 cảm biến OK → LED sáng
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LOW = ON
    } 
    else 
    {
      // Không có cảm biến nào → LED nhấp nháy
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

**API sử dụng:** CMSIS-RTOS v2 (lớp trừu tượng trên FreeRTOS)

### 3.2. Cấu hình FreeRTOS

File: `FreeRTOSConfig.h`

```c
// 1. TICK RATE
#define configTICK_RATE_HZ                       ((TickType_t)1000)
// => 1 tick = 1 ms (vSysTickHandler được gọi mỗi 1ms)

// 2. CPU FREQUENCY
#define configCPU_CLOCK_HZ                       ((uint32_t)100000000)
// => STM32F411CE @ 100 MHz

// 3. HEAP SIZE
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
// => 15 KB RAM cho tasks, queues, mutexes

// 4. PRIORITIES
#define configMAX_PRIORITIES                     ( 56 )
// => Priority từ 0 đến 55

// 5. PREEMPTION
#define configUSE_PREEMPTION                     1
// => Task ưu tiên cao chiếm quyền từ task thấp

// 6. MUTEXES
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1

// 7. TIMERS
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 3 )

// 8. HEAP IMPLEMENTATION
#define USE_FreeRTOS_HEAP_4
// => heap_4.c:  Cho phép malloc/free, chống phân mảnh

// 9. NEWLIB SUPPORT
#define configUSE_NEWLIB_REENTRANT               1
// => Thread-safe cho printf, malloc, etc. 
```

### 3.3. Các Task trong hệ thống

Hệ thống có **4 tasks chính**:

| Task Name             | Priority         | Stack Size | Period  | Mục đích                           |
|-----------------------|------------------|------------|---------|------------------------------------|
| SensorReadingTask     | osPriorityHigh   | 512 words  | 3000 ms | Đọc cảm biến BME280, CCS811        |
| UartCommunicationTask | osPriorityAboveNormal | 768 words | Event-driven | Truyền dữ liệu qua UART |
| LedStatusTask         | osPriorityBelowNormal | 256 words | 500 ms  | Điều khiển LED trạng thái         |
| SystemMonitorTask     | osPriorityLow    | 512 words  | 60000 ms | Gửi heartbeat, monitor system     |

---

#### **Task 1: SensorReadingTask (Priority: High)**

**Mục đích:** Đọc dữ liệu từ cảm biến (nhiệm vụ thời gian thực quan trọng nhất)

**Code:**
```c
void SensorReadingTask(void *argument)
{
  SensorData_t sensorData;
  uint8_t bme_init_ok = 0;
  uint8_t ccs_init_ok = 0;
  
  // Khởi tạo cảm biến
  bme_init_ok = (BME280_Init(&hi2c2) == HAL_OK);
  ccs_init_ok = (CCS811_Init(&hi2c2) == HAL_OK);
  
  // Infinite loop
  for(;;)
  {
    // Acquire I2C mutex (timeout 1000ms)
    if (osMutexAcquire(i2cMutexHandle, 1000) == osOK) 
    {
      // === CRITICAL SECTION ===
      
      // Đọc BME280
      if (bme_init_ok) {
        if (BME280_ReadAll(&hi2c2, &sensorData.temperature, 
                           &sensorData. humidity, 
                           &sensorData. pressure) == HAL_OK) {
          sensorData.bme280_status = 1;
          systemStatus.bme280_ok = 1;
        } else {
          sensorData.bme280_status = 0;
          systemStatus.bme280_ok = 0;
          // Retry init
          bme_init_ok = (BME280_Init(&hi2c2) == HAL_OK);
        }
      }
      
      // Đọc CCS811
      if (ccs_init_ok) {
        if (CCS811_ReadData(&hi2c2, &sensorData.co2, &sensorData.tvoc) == HAL_OK) {
          sensorData.ccs811_status = 1;
          systemStatus.ccs811_ok = 1;
          
          // Environmental compensation
          CCS811_SetEnvironmentalData(sensorData.temperature, 
                                      sensorData.humidity);
        } else {
          sensorData.ccs811_status = 0;
          systemStatus.ccs811_ok = 0;
          // Retry init
          ccs_init_ok = (CCS811_Init(&hi2c2) == HAL_OK);
        }
      }
      
      // Release mutex
      osMutexRelease(i2cMutexHandle);
      
      // === END CRITICAL SECTION ===
      
      // Gửi dữ liệu vào queue (timeout 100ms)
      sensorData.timestamp = osKernelGetTickCount();
      osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
      
      systemStatus.error_count = 0; // Reset error count nếu thành công
    } 
    else 
    {
      // Timeout khi acquire mutex
      systemStatus. error_count++;
    }
    
    // Delay 3 giây
    osDelay(3000);
  }
}
```

**Tại sao priority HIGH?**
- Cảm biến cần đọc đúng thời điểm (real-time requirement)
- Nếu bỏ lỡ, dữ liệu sẽ không chính xác
- Không thể bị gián đoạn bởi tasks khác

**Stack size:  512 words (2048 bytes)**
- Task stack lưu:  local variables, function calls, interrupt context
- BME280/CCS811 driver functions cần ~1. 5KB stack
- 512 words = 2048 bytes là đủ

---

#### **Task 2: UartCommunicationTask (Priority: Above Normal)**

**Mục đích:** Nhận dữ liệu từ queue và truyền qua UART

**Code:**
```c
void UartCommunicationTask(void *argument)
{
  SensorData_t sensorData;
  char uartBuffer[256];
  uint8_t format = FORMAT_JSON; // Mặc định JSON
  
  for(;;)
  {
    // Chờ dữ liệu từ queue (blocking, timeout 5000ms)
    if (osMessageQueueGet(sensorDataQueueHandle, &sensorData, NULL, 5000) == osOK)
    {
      int len = 0;
      
      // Format dữ liệu theo định dạng
      switch(format) {
        case FORMAT_JSON:
          len = snprintf(uartBuffer, sizeof(uartBuffer),
            "{\"device\":\"STM32F411_RTOS\","
            "\"timestamp\":%lu,"
            "\"temp\":%.2f,"
            "\"humi\":%.2f,"
            "\"pres\":%. 2f,"
            "\"co2\":%d,"
            "\"tvoc\":%d,"
            "\"bme\":\"%s\","
            "\"ccs\":\"%s\"}\r\n",
            sensorData.timestamp,
            sensorData.temperature,
            sensorData.humidity,
            sensorData.pressure,
            sensorData.co2,
            sensorData.tvoc,
            sensorData.bme280_status ? "OK" : "ERROR",
            sensorData.ccs811_status ? "OK" :  "ERROR"
          );
          break;
          
        case FORMAT_CSV: 
          len = snprintf(uartBuffer, sizeof(uartBuffer),
            "%lu,%.2f,%.2f,%.2f,%d,%d\r\n",
            sensorData.timestamp,
            sensorData.temperature,
            sensorData.humidity,
            sensorData.pressure,
            sensorData.co2,
            sensorData.tvoc
          );
          break;
          
        case FORMAT_PLAIN:
          len = snprintf(uartBuffer, sizeof(uartBuffer),
            "TEMP: %.2f HUMI:%.2f PRES:%.2f CO2:%d TVOC:%d\r\n",
            sensorData.temperature,
            sensorData.humidity,
            sensorData.pressure,
            sensorData.co2,
            sensorData.tvoc
          );
          break;
      }
      
      // Acquire UART mutex (timeout 1000ms)
      if (osMutexAcquire(uartMutexHandle, 1000) == osOK)
      {
        // Truyền qua UART
        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, len, 100);
        
        // Release mutex
        osMutexRelease(uartMutexHandle);
      }
    }
    
    // Xử lý command từ UART (nếu có)
    // ...  (code xử lý command)
  }
}
```

**Tại sao priority ABOVE NORMAL?**
- Truyền dữ liệu UART cần kịp thời nhưng không quan trọng bằng đọc sensor
- Nếu chậm một chút không ảnh hưởng lớn đến hệ thống

**Stack size: 768 words (3072 bytes)**
- `snprintf()` cần nhiều stack cho formatting
- UART buffer 256 bytes
- Function call overhead

---

#### **Task 3: LedStatusTask (Priority:  Below Normal)**

**Mục đích:** Cập nhật trạng thái LED

**Code:**
```c
void LedStatusTask(void *argument)
{
  for(;;)
  {
    // Kiểm tra trạng thái cảm biến
    if (systemStatus.bme280_ok || systemStatus. ccs811_ok) 
    {
      // Ít nhất 1 cảm biến OK → LED sáng liên tục
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Active LOW
    } 
    else 
    {
      // Không có cảm biến nào → LED nhấp nháy
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    
    osDelay(500); // 500ms
  }
}
```

**Tại sao priority BELOW NORMAL?**
- LED chỉ là indicator, không quan trọng
- Có thể bị delay một chút mà không ảnh hưởng

**Stack size: 256 words (1024 bytes)**
- Task đơn giản, không cần nhiều stack

---

#### **Task 4: SystemMonitorTask (Priority: Low)**

**Mục đích:** Giám sát hệ thống, gửi heartbeat

**Code:**
```c
void SystemMonitorTask(void *argument)
{
  char heartbeatMsg[128];
  
  for(;;)
  {
    // Tạo heartbeat message
    int len = snprintf(heartbeatMsg, sizeof(heartbeatMsg),
      "{\"type\":\"HEARTBEAT\","
      "\"uptime\":%lu,"
      "\"bme280\":%s,"
      "\"ccs811\":%s,"
      "\"errors\":%d}\r\n",
      osKernelGetTickCount() / 1000, // Uptime in seconds
      systemStatus. bme280_ok ? "true" : "false",
      systemStatus.ccs811_ok ?  "true" : "false",
      systemStatus.error_count
    );
    
    // Acquire UART mutex
    if (osMutexAcquire(uartMutexHandle, 1000) == osOK)
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)heartbeatMsg, len, 100);
      osMutexRelease(uartMutexHandle);
    }
    
    // Kiểm tra stack usage (optional)
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    
    osDelay(60000); // 60 seconds
  }
}
```

**Tại sao priority LOW? **
- Heartbeat không cần real-time
- Có thể delay vài giây mà không ảnh hưởng

---

### 3.4. Cơ chế Scheduling

**Preemptive Priority-based Scheduling:**

```
Time ───────────────────────────────────────────────►

Priority HIGH   │█████│     │█████│     │█████│
(Sensor)        │     │     │     │     │     │
                │     │     │     │     │     │
Priority ABOVE  │     │█████│     │█████│     │█████
(UART)          │     │     │     │     │     │
                │     │     │     │     │     │
Priority BELOW  │     │     │█████     │     │     │
(LED)           │     │     │     │     │     │
                │     │     │     │     │     │
Priority LOW    │     │     │     │█████│     │     │
(Monitor)       │     │     │     │     │     │     │

Legend:
█ = Task đang chạy
│ = Task bị preempt hoặc đang blocked
```

**Ví dụ:**
1. LED Task đang chạy (priority low)
2. Sensor Task ready → Preempt LED Task → Sensor chạy
3. Sensor xong → LED tiếp tục

---

### 3.5. Task States

```
                      ┌─────────────┐
         Create ──────► NOT RUNNING │
                      └──────┬──────┘
                             │ Ready
                             ▼
         ┌──────────────────────────────────┐
         │                                  │
         │         ┌───────────┐            │
 Block ◄─┼─────────┤  RUNNING  │────────────┼─► Suspend
         │         └───────────┘            │
         │             ▲     │              │
         │             │     │              │
         │      Resume │     │ Delay/Block  │
         │             │     ▼              │
         │         ┌───────────┐            │
         │         │  BLOCKED  │            │
         │         └───────────┘            │
         │                                  │
         └──────────────────────────────────┘
```

**States:**
- **Running**: Task đang được CPU thực thi
- **Ready**: Task sẵn sàng chạy, chờ CPU
- **Blocked**: Task đang chờ event (delay, queue, mutex, etc.)
- **Suspended**: Task bị tạm dừng (không được schedule)

---

## 4. FREERTOS - QUEUE, SEMAPHORE, MUTEX

### 4.1. Queue (Hàng đợi)

**Mục đích:** Inter-task communication - truyền dữ liệu giữa các tasks

#### Lý thuyết

**Queue** là cấu trúc FIFO (First-In-First-Out) thread-safe. 

**Đặc điểm:**
- Tasks có thể send/receive data
- Blocking send/receive với timeout
- Multiple readers/writers
- Fixed size items

#### Code ví dụ

```c
// 1. Khai báo handle
osMessageQueueId_t sensorDataQueueHandle;

// 2. Định nghĩa data structure
typedef struct {
  uint32_t timestamp;
  float temperature;
  float humidity;
  float pressure;
  uint16_t co2;
  uint16_t tvoc;
  uint8_t bme280_status;
  uint8_t ccs811_status;
} SensorData_t;

// 3. Tạo queue trong main()
void MX_FREERTOS_Init(void)
{
  // Tạo queue với 5 phần tử, mỗi phần tử sizeof(SensorData_t)
  sensorDataQueueHandle = osMessageQueueNew(5, sizeof(SensorData_t), NULL);
  
  if (sensorDataQueueHandle == NULL) {
    Error_Handler();
  }
}

// 4. Gửi dữ liệu vào queue (Producer)
void SensorReadingTask(void *argument)
{
  SensorData_t sensorData;
  
  for(;;)
  {
    // Đọc dữ liệu... 
    
    // Send vào queue (priority = 0, timeout = 100ms)
    osStatus_t status = osMessageQueuePut(sensorDataQueueHandle, 
                                          &sensorData, 
                                          0,    // priority
                                          100); // timeout (ms)
    
    if (status != osOK) {
      // Queue full hoặc timeout
      systemStatus.error_count++;
    }
    
    osDelay(3000);
  }
}

// 5. Nhận dữ liệu từ queue (Consumer)
void UartCommunicationTask(void *argument)
{
  SensorData_t sensorData;
  
  for(;;)
  {
    // Get từ queue (blocking, timeout = 5000ms)
    osStatus_t status = osMessageQueueGet(sensorDataQueueHandle, 
                                          &sensorData, 
                                          NULL,  // priority (output)
                                          5000); // timeout (ms)
    
    if (status == osOK) {
      // Có dữ liệu → Xử lý
      SendDataToUart(&sensorData);
    } else {
      // Timeout → Không có dữ liệu trong 5 giây
    }
  }
}
```

#### Queue Parameters

```c
osMessageQueueNew(
  uint32_t msg_count,    // Số lượng message tối đa
  uint32_t msg_size,     // Kích thước mỗi message (bytes)
  const osMessageQueueAttr_t *attr  // NULL = default
)

osMessageQueuePut(
  osMessageQueueId_t mq_id,  // Queue handle
  const void *msg_ptr,        // Pointer to data
  uint8_t msg_prio,           // Priority (0 = lowest)
  uint32_t timeout            // Timeout in ticks (0 = no wait, osWaitForever)
)

osMessageQueueGet(
  osMessageQueueId_t mq_id,  // Queue handle
  void *msg_ptr,              // Pointer to buffer
  uint8_t *msg_prio,          // Priority (output, NULL = ignore)
  uint32_t timeout            // Timeout in ticks
)
```

#### Memory Usage

```
Queue Memory = msg_count × (msg_size + overhead)
             = 5 × (64 bytes + ~20 bytes)
             ≈ 420 bytes
```

---

### 4.2. Mutex (Mutual Exclusion)

**Mục đích:** Bảo vệ shared resources (I2C, UART, variables)

#### Lý thuyết

**Mutex** = Binary semaphore với ownership và priority inheritance.

**Đặc điểm:**
- Chỉ task nào acquire mới được release (ownership)
- Priority inheritance:  Tránh priority inversion
- Recursive mutex: Task có thể acquire nhiều lần

**Priority Inversion Problem:**
```
Task H (High)    :  Cần acquire mutex
Task M (Medium)  : Đang chạy
Task L (Low)     : Đang giữ mutex

→ Task H bị block bởi Task L
→ Nhưng Task L bị preempt bởi Task M
→ Task H phải chờ Task M xong → Priority inversion! 

Solution: Priority Inheritance
→ Task L tạm thời được nâng priority = Task H
→ Task L chạy xong → Release mutex → Task H chạy
```

#### Code ví dụ - I2C Mutex

```c
// 1. Khai báo handle
osMutexId_t i2cMutexHandle;

// 2. Tạo mutex trong main()
void MX_FREERTOS_Init(void)
{
  // Tạo mutex với priority inheritance
  const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex",
    .attr_bits = osMutexPrioInherit, // Kích hoạt priority inheritance
    .cb_mem = NULL,
    . cb_size = 0U
  };
  
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);
  
  if (i2cMutexHandle == NULL) {
    Error_Handler();
  }
}

// 3. Sử dụng trong Task
void SensorReadingTask(void *argument)
{
  for(;;)
  {
    // Acquire mutex (timeout 1000ms)
    if (osMutexAcquire(i2cMutexHandle, 1000) == osOK) 
    {
      // ========== CRITICAL SECTION ==========
      // Chỉ 1 task được vào đây tại 1 thời điểm
      
      // Đọc BME280 qua I2C
      HAL_I2C_Mem_Read(&hi2c2, BME280_ADDR, 0xF7, 1, buffer, 8, 100);
      
      // Đọc CCS811 qua I2C
      HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDR, 0x02, 1, buffer, 8, 100);
      
      // ========== END CRITICAL SECTION ==========
      
      // Release mutex (QUAN TRỌNG!)
      osMutexRelease(i2cMutexHandle);
    } 
    else 
    {
      // Timeout → Có lỗi (deadlock hoặc task khác giữ quá lâu)
      systemStatus.error_count++;
    }
    
    osDelay(3000);
  }
}
```

#### Code ví dụ - UART Mutex

```c
// Global mutex cho UART
osMutexId_t uartMutexHandle;

// Task 1: UartCommunicationTask gửi sensor data
void UartCommunicationTask(void *argument)
{
  for(;;)
  {
    // ...  nhận dữ liệu từ queue ...
    
    if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
      HAL_UART_Transmit(&huart2, sensorDataBuffer, len, 100);
      osMutexRelease(uartMutexHandle);
    }
  }
}

// Task 2: SystemMonitorTask gửi heartbeat
void SystemMonitorTask(void *argument)
{
  for(;;)
  {
    if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
      HAL_UART_Transmit(&huart2, heartbeatBuffer, len, 100);
      osMutexRelease(uartMutexHandle);
    }
    
    osDelay(60000);
  }
}
```

#### Mutex Best Practices

```c
// ✅ ĐÚNG:
if (osMutexAcquire(mutexHandle, timeout) == osOK) {
  // Critical section
  DoWork();
  osMutexRelease(mutexHandle); // Luôn release
}

// ❌ SAI:  Quên release
osMutexAcquire(mutexHandle, timeout);
DoWork();
// Lỗi:  Không release → Deadlock! 

// ❌ SAI: Release trong ISR
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  osMutexRelease(mutexHandle); // LỖI:  Không được release trong ISR! 
}

// ✅ ĐÚNG:  Xử lý error case
if (osMutexAcquire(mutexHandle, timeout) == osOK) {
  HAL_StatusTypeDef status = DoWork();
  
  if (status != HAL_OK) {
    // Dù có lỗi vẫn phải release
    osMutexRelease(mutexHandle);
    return ERROR;
  }
  
  osMutexRelease(mutexHandle);
}
```

---

### 4.3. Semaphore

**Lưu ý:** Dự án này **KHÔNG** sử dụng semaphore, chỉ dùng mutex.

#### Lý thuyết Semaphore

**Binary Semaphore:**
- Giá trị: 0 hoặc 1
- Dùng cho synchronization giữa tasks hoặc ISR → Task
- Không có ownership (bất kỳ task nào cũng có thể signal/wait)

**Counting Semaphore:**
- Giá trị: 0 đến N
- Dùng cho resource counting (ví dụ: 3 buffer slots)

#### Mutex vs Semaphore

| Đặc điểm                | Mutex                     | Semaphore                  |
|-------------------------|---------------------------|----------------------------|
| **Ownership**           | Có (task acquire = task release) | Không (bất kỳ task nào cũng signal/wait) |
| **Priority Inheritance**| Có                        | Không                      |
| **Use case**            | Mutual exclusion (bảo vệ resource) | Synchronization (đồng bộ tasks) |
| **Value**               | Binary (locked/unlocked)  | Binary hoặc Counting       |
| **ISR usage**           | Không (chỉ task)          | Có (ISR có thể signal)     |

**Ví dụ Semaphore:**
```c
// Binary Semaphore để đồng bộ ISR → Task
osSemaphoreId_t adcSemaphoreHandle;

// Tạo semaphore
adcSemaphoreHandle = osSemaphoreNew(1, 0, NULL); // max=1, initial=0

// ISR: ADC conversion complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  osSemaphoreRelease(adcSemaphoreHandle); // Signal task
}

// Task: Chờ ADC complete
void AdcTask(void *argument)
{
  for(;;)
  {
    // Start ADC conversion
    HAL_ADC_Start_IT(&hadc1);
    
    // Chờ ISR signal (blocking)
    osSemaphoreAcquire(adcSemaphoreHandle, osWaitForever);
    
    // Process ADC result
    uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
  }
}
```

**Tại sao dự án này không dùng semaphore?**
- Không cần synchronization giữa ISR và Task
- Chỉ cần bảo vệ shared resources (I2C, UART) → Dùng mutex
- Mutex có priority inheritance → An toàn hơn

---

## 5. CẤU HÌNH HỆ THỐNG

### 5.1. Pin Configuration

```
┌─────────────────────────────────────────────────┐
│             STM32F411CEU6 (Black Pill)          │
├─────────────────────────────────────────────────┤
│                                                 │
│  PC13 ────► LED (Active LOW)                   │
│                                                 │
│  PB3  ────► I2C2_SDA ───► BME280 + CCS811      │
│  PB10 ────► I2C2_SCL ───┘                      │
│                                                 │
│  PA2  ────► USART2_TX ──► Orange Pi 4A         │
│  PA3  ────► USART2_RX ◄── Orange Pi 4A         │
│                                                 │
│  GND  ────► Common Ground                      │
│  3V3  ────► 3.3V Power                         │
└─────────────────────────────────────────────────┘
```

### 5.2. Clock Configuration

```c
// System Clock:  100 MHz
// - HSE: 25 MHz (External Crystal)
// - PLL: 100 MHz
// - AHB: 100 MHz
// - APB1: 50 MHz (UART2, I2C2)
// - APB2: 100 MHz

// STM32CubeMX Configuration: 
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct. PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL. PLLM = 25;
RCC_OscInitStruct.PLL.PLLN = 200;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 200/2 = 100 MHz
```

### 5.3. Memory Usage

**Flash (512 KB):**
```
Application code      : ~80 KB
FreeRTOS kernel       : ~10 KB
HAL drivers           : ~30 KB
Sensor drivers        : ~15 KB
───────────────────────────────
Total used            : ~135 KB
Free                  : ~377 KB
```

**RAM (128 KB):**
```
Global variables      : ~2 KB
Heap (FreeRTOS)       : 15 KB
  - Tasks             : ~10.7 KB
  - Queues            : ~2.9 KB
  - Mutexes           : ~0.3 KB
  - Overhead          : ~0.5 KB
Stack (. stack)        : ~3 KB (main stack, ISR stack)
───────────────────────────────
Total used            : ~20 KB
Free                  : ~108 KB
```

### 5.4. Initialization Sequence

```c
int main(void)
{
  // 1. Reset peripherals
  HAL_Init();
  
  // 2. Configure system clock (100 MHz)
  SystemClock_Config();
  
  // 3. Initialize GPIO, I2C, UART
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  
  // 4. Initialize FreeRTOS
  osKernelInitialize();
  
  // 5. Create mutexes
  i2cMutexHandle = osMutexNew(NULL);
  uartMutexHandle = osMutexNew(NULL);
  
  // 6. Create queues
  sensorDataQueueHandle = osMessageQueueNew(5, sizeof(SensorData_t), NULL);
  
  // 7. Create tasks
  sensorTaskHandle = osThreadNew(SensorReadingTask, NULL, &sensorTask_attributes);
  uartTaskHandle = osThreadNew(UartCommunicationTask, NULL, &uartTask_attributes);
  ledTaskHandle = osThreadNew(LedStatusTask, NULL, &ledTask_attributes);
  monitorTaskHandle = osThreadNew(SystemMonitorTask, NULL, &monitorTask_attributes);
  
  // 8. Start scheduler
  osKernelStart(); // Không bao giờ return
  
  // 9. Không bao giờ đến đây
  while (1) {}
}
```

---

## 6. HƯỚNG DẪN SỬ DỤNG

### 6.1. Build và Flash

**Yêu cầu:**
- STM32CubeIDE 1.11+
- STM32CubeMX (đã tích hợp trong IDE)
- ST-Link V2 hoặc tương đương

**Các bước:**
```bash
# 1. Clone repository
git clone https://github.com/n22dcdt066-toan/STM32-BME280-CCS811-CBDAD-OLED-PREERTOS.git
cd STM32-BME280-CCS811-CBDAD-OLED-PREERTOS

# 2.  Mở project trong STM32CubeIDE
# File → Open Projects from File System → Select folder

# 3. Build project
# Project → Build All (Ctrl+B)
# Hoặc chọn "Release" configuration để tối ưu code

# 4. Flash lên STM32F411CEU6
# Run → Debug (F11)
# Hoặc:  Run → Run (Ctrl+F11)
```

### 6.2. Kết nối phần cứng

```
STM32F411CEU6          BME280 (I2C)
─────────────          ────────────
PB3 (SDA)    ────────  SDA
PB10 (SCL)   ────────  SCL
3V3          ────────  VCC
GND          ────────  GND

STM32F411CEU6          CCS811 (I2C)
─────────────          ────────────
PB3 (SDA)    ────────  SDA
PB10 (SCL)   ────────  SCL
3V3          ────────  VCC
GND          ────────  GND
GND          ────────  WAK (Wake pin)

STM32F411CEU6          Orange Pi 4A
─────────────          ────────────
PA2 (TX)     ────────  RX
PA3 (RX)     ────────  TX
GND          ────────  GND

Note: I2C cần pull-up resistors 4.7kΩ trên SDA và SCL
```

### 6.3. Kiểm tra output

**Terminal Serial (115200 baud):**
```bash
# Linux/Mac: 
screen /dev/ttyUSB0 115200

# Windows:
# Sử dụng PuTTY hoặc TeraTerm
# COM port: COMx, Baud: 115200

# Python script:
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

**Output mẫu:**
```json
{"type":"SYSTEM_START","device":"STM32F411_RTOS"}
{"type":"INIT","bme280": true,"ccs811":true}
{"device":"STM32F411_RTOS","timestamp": 3000,"temp":25.60,"humi":65.20,"pres":1013.20,"co2":400,"tvoc":25,"bme":"OK","ccs":"OK"}
{"device":"STM32F411_RTOS","timestamp":6000,"temp":25.62,"humi":65.18,"pres":1013.18,"co2":401,"tvoc":26,"bme":"OK","ccs":"OK"}
{"type":"HEARTBEAT","uptime":60,"bme280":true,"ccs811":true,"errors":0}
```

### 6.4. Gửi command qua UART

**Command format:** `COMMAND\r\n`

```bash
# Lấy trạng thái hệ thống
GET_STATUS

# Đổi format output
SET_FORMAT_JSON
SET_FORMAT_CSV
SET_FORMAT_PLAIN

# Restart hệ thống
RESTART
```

### 6.5. Debug với SEGGER SystemView (Optional)

**Cấu hình:**
```c
// FreeRTOSConfig.h
#define INCLUDE_xTaskGetIdleTaskHandle       1
#define configUSE_TRACE_FACILITY             1
#define configGENERATE_RUN_TIME_STATS        1

// main.c
#include "SEGGER_SYSVIEW. h"

// Trong các task
SEGGER_SYSVIEW_Print("SensorReadingTask:  BME280 OK");
```

**Visualization:**
- Task timeline
- CPU usage per task
- Queue/Mutex events
- Context switches

---

## 7. TROUBLESHOOTING

### 7.1. Cảm biến không hoạt động

**Triệu chứng:** `"bme":"ERROR"` hoặc `"ccs":"ERROR"`

**Giải pháp:**
1. Kiểm tra kết nối I2C (SDA, SCL, VCC, GND)
2. Kiểm tra pull-up resistors 4.7kΩ trên SDA/SCL
3. Kiểm tra địa chỉ I2C với `i2cdetect` (nếu có logic analyzer)
4. Thử thay đổi địa chỉ trong code: 
```c
// BME280: 0x76 hoặc 0x77
#define BME280_ADDR (0x76 << 1)

// CCS811: 0x5A hoặc 0x5B
#define CCS811_ADDR (0x5A << 1)
```

### 7.2. Heap overflow

**Triệu chứng:** `vApplicationMallocFailedHook()` được gọi, system hang

**Giải pháp:**
1. Tăng `configTOTAL_HEAP_SIZE` trong `FreeRTOSConfig.h`
```c
#define configTOTAL_HEAP_SIZE  ((size_t)20480) // 20 KB
```
2. Giảm stack size của các tasks
3. Kiểm tra memory leak

### 7.3. Stack overflow

**Triệu chứng:** `vApplicationStackOverflowHook()` được gọi

**Giải pháp:**
1. Tăng stack size của task bị overflow
```c
const osThreadAttr_t sensorTask_attributes = {
  .stack_size = 1024 * 4, // Tăng từ 512 lên 1024 words
};
```
2. Kiểm tra local variables quá lớn trong task
3. Tránh recursion trong tasks

### 7.4. Priority inversion

**Triệu chứng:** Task priority cao bị delay bởi task priority thấp

**Giải pháp:**
Đảm bảo mutex có priority inheritance:
```c
const osMutexAttr_t mutex_attributes = {
  .attr_bits = osMutexPrioInherit, // QUAN TRỌNG! 
};
mutexHandle = osMutexNew(&mutex_attributes);
```

---

## 8. TÀI LIỆU THAM KHẢO

1. **FreeRTOS Official Documentation**
   - https://www.freertos.org/Documentation/RTOS_book.html
   - Mastering the FreeRTOS Real Time Kernel (PDF)

2. **STM32F411CEU6 Datasheet**
   - https://www.st.com/resource/en/datasheet/stm32f411ce. pdf

3. **BME280 Datasheet**
   - https://www.bosch-sensortec. com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf

4. **CCS811 Datasheet**
   - https://www.sciosense.com/wp-content/uploads/2020/01/CCS811-Datasheet. pdf

5. **CMSIS-RTOS v2 API**
   - https://arm-software.github.io/CMSIS_5/RTOS2/html/index.html

---

## 9. LIÊN HỆ

**Sinh viên:** N22DCDT066 - Toàn  
**Repository:** https://github.com/n22dcdt066-toan/STM32-BME280-CCS811-CBDAD-OLED-PREERTOS

---

## PHỤ LỤC:  CÂU HỎI VIVA THƯỜNG GẶP

### Câu hỏi về hệ thống

**Q1: Giới thiệu chung về dự án của bạn? **
> Dự án sử dụng STM32F411CEU6 với FreeRTOS để đọc dữ liệu từ cảm biến BME280 (nhiệt độ, độ ẩm, áp suất) và CCS811 (CO2, TVOC) qua I2C, sau đó truyền dữ liệu qua UART đến Orange Pi 4A.  Hệ thống có 4 tasks chạy song song:  SensorReadingTask đọc cảm biến, UartCommunicationTask truyền dữ liệu, LedStatusTask báo trạng thái, và SystemMonitorTask giám sát hệ thống.

**Q2: Tại sao sử dụng FreeRTOS thay vì bare-metal?**
> FreeRTOS cho phép xử lý đa nhiệm hiệu quả hơn.  Với bare-metal, phải dùng superloop và polling, khó kiểm soát timing. FreeRTOS cung cấp preemptive scheduling, đảm bảo task ưu tiên cao (đọc sensor) luôn được thực thi kịp thời, đồng thời có cơ chế IPC (queue, mutex) để đồng bộ dữ liệu an toàn giữa các tasks.

### Câu hỏi về giao tiếp ngoại vi

**Q3: Giải thích giao tiếp

