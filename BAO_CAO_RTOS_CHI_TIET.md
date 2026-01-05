# BÁO CÁO CHI TIẾT: CÁCH VIẾT CHƯƠNG TRÌNH RTOS TRÊN STM32

**Tác giả:** AI Assistant  
**Ngày:** 27/10/2025  
**Project:** STM32-BME280-CCS811-PREERTOS  
**Vi điều khiển:** STM32F411CEU6  
**RTOS:** FreeRTOS v10.3.1 với CMSIS-RTOS v2 API

---

## MỤC LỤC

1. [Tổng quan về FreeRTOS](#1-tổng-quan-về-freertos)
2. [Cấu trúc project RTOS](#2-cấu-trúc-project-rtos)
3. [Cấu hình FreeRTOS](#3-cấu-hình-freertos)
4. [Kiến trúc hệ thống](#4-kiến-trúc-hệ-thống)
5. [Tasks (Nhiệm vụ)](#5-tasks-nhiệm-vụ)
6. [Queues (Hàng đợi)](#6-queues-hàng-đợi)
7. [Mutexes (Khóa tương hỗ)](#7-mutexes-khóa-tương-hỗ)
8. [Đồng bộ hóa và chia sẻ tài nguyên](#8-đồng-bộ-hóa-và-chia-sẻ-tài-nguyên)
9. [Quản lý bộ nhớ](#9-quản-lý-bộ-nhớ)
10. [Xử lý lỗi và debug](#10-xử-lý-lỗi-và-debug)
11. [Best Practices (Thực hành tốt nhất)](#11-best-practices)
12. [Kết luận](#12-kết-luận)

---

## 1. TỔNG QUAN VỀ FREERTOS

### 1.1. FreeRTOS là gì?

FreeRTOS (Free Real-Time Operating System) là một hệ điều hành thời gian thực miễn phí, mã nguồn mở được thiết kế cho các hệ thống nhúng. Nó cho phép:

- **Đa nhiệm (Multitasking)**: Chạy nhiều tasks đồng thời
- **Lập lịch ưu tiên (Priority-based Scheduling)**: Tasks có mức ưu tiên cao được ưu tiên thực thi
- **Đồng bộ hóa (Synchronization)**: Queues, Semaphores, Mutexes
- **Quản lý thời gian**: Software Timers, Delays
- **Tiết kiệm năng lượng**: Chế độ Idle và Tick-less mode

### 1.2. CMSIS-RTOS v2 API

Project này sử dụng **CMSIS-RTOS v2** - một wrapper chuẩn hóa của ARM cho FreeRTOS. Ưu điểm:

- API chuẩn ARM, dễ chuyển đổi giữa các RTOS khác nhau
- Hỗ trợ tốt hơn từ STM32CubeMX
- Namespace rõ ràng (các hàm bắt đầu bằng `os`)

**Ví dụ so sánh:**

| FreeRTOS Native | CMSIS-RTOS v2 |
|----------------|---------------|
| `xTaskCreate()` | `osThreadNew()` |
| `xQueueCreate()` | `osMessageQueueNew()` |
| `vTaskDelay()` | `osDelay()` |
| `xSemaphoreCreateMutex()` | `osMutexNew()` |

---

## 2. CẤU TRÚC PROJECT RTOS

### 2.1. Sơ đồ tổng thể

```
STM32-BME280-CCS811-PREERTOS/
├── Core/
│   ├── Inc/
│   │   ├── FreeRTOSConfig.h    <- Cấu hình RTOS quan trọng
│   │   ├── main.h
│   │   └── các file header sensor
│   └── Src/
│       ├── main.c               <- File chính chứa logic RTOS
│       ├── freertos.c           <- RTOS hooks và callbacks
│       └── các file driver sensor
├── Middlewares/
│   └── Third_Party/
│       └── FreeRTOS/            <- Kernel FreeRTOS
└── Drivers/
    └── STM32F4xx_HAL_Driver/    <- HAL Library
```

### 2.2. File quan trọng

#### **FreeRTOSConfig.h**
Chứa tất cả cấu hình RTOS (heap size, tick rate, priorities, v.v.)

#### **main.c**
- Khởi tạo phần cứng (HAL)
- Tạo Tasks, Queues, Mutexes
- Khởi động RTOS scheduler
- Chứa code của các Task functions

#### **freertos.c**
- Các hook functions (malloc failed, idle hook, tick hook)
- Không chứa logic chính

---

## 3. CẤU HÌNH FREERTOS

### 3.1. FreeRTOSConfig.h - Các thông số quan trọng

```c
// File: Core/Inc/FreeRTOSConfig.h

// 1. TICK RATE - Tần số ngắt hệ thống
#define configTICK_RATE_HZ                       ((TickType_t)1000)
// => 1000 Hz = 1 tick mỗi 1 ms
// Đơn vị thời gian nhỏ nhất trong osDelay() là 1 ms

// 2. CPU CLOCK
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
// => 96 MHz cho STM32F411

// 3. HEAP SIZE - Bộ nhớ dành cho RTOS
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
// => 15 KB RAM cho tasks, queues, mutexes
// QUAN TRỌNG: Phải đủ lớn để chứa tất cả tasks + queues + overhead

// 4. PRIORITIES
#define configMAX_PRIORITIES                     ( 56 )
// => Có thể dùng từ priority 0 đến 55
// Priority càng cao, task càng ưu tiên

// 5. STACK SIZE
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
// => Stack tối thiểu cho idle task (128 words = 512 bytes)

// 6. PREEMPTION
#define configUSE_PREEMPTION                     1
// => Cho phép task ưu tiên cao chiếm quyền từ task ưu tiên thấp

// 7. MUTEXES
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
// => Kích hoạt Mutexes và Recursive Mutexes

// 8. TIMERS
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 3 )
#define configTIMER_TASK_STACK_DEPTH             256
// => Kích hoạt Software Timers

// 9. HEAP IMPLEMENTATION
#define USE_FreeRTOS_HEAP_4
// => Sử dụng heap_4.c - cho phép cấp phát và giải phóng động

// 10. NEWLIB SUPPORT
#define configUSE_NEWLIB_REENTRANT          1
// => Thread-safe cho các hàm C standard library (malloc, printf...)
```

### 3.2. Giải thích chi tiết các tham số

#### **a. configTICK_RATE_HZ**
- Định nghĩa tần số System Tick (SysTick interrupt)
- 1000 Hz nghĩa là có 1000 interrupts/giây → 1 tick = 1 ms
- Ảnh hưởng đến `osDelay()`: `osDelay(100)` = delay 100 ms

#### **b. configTOTAL_HEAP_SIZE**
Công thức tính heap cần thiết:
```
Heap = Σ(Task Stack Sizes) + Σ(Queue Sizes) + Overhead
```

Trong project này:
```
Tasks:
- SensorTask:   512 words × 4 = 2048 bytes
- UartTask:     768 words × 4 = 3072 bytes  
- LedTask:      256 words × 4 = 1024 bytes
- MonitorTask:  512 words × 4 = 2048 bytes
- OledTask:     512 words × 4 = 2048 bytes
- DefaultTask:  128 words × 4 = 512 bytes
Tổng Tasks: ~10.7 KB

Queues:
- sensorDataQueue: 5 × sizeof(SensorData_t) ≈ 5 × 64 = 320 bytes
- uartTxQueue:     10 × sizeof(UartMessage_t) ≈ 10 × 258 = 2580 bytes
Tổng Queues: ~2.9 KB

Mutexes: ~300 bytes
Overhead FreeRTOS: ~500 bytes

Tổng: ~14.4 KB → Cấu hình 15 KB là đủ
```

#### **c. Priority Levels**
```
Priority 0 = Thấp nhất
Priority 55 = Cao nhất (trong config này)

Idle Task = Priority 0 (tự động tạo bởi RTOS)
```

---

## 4. KIẾN TRÚC HỆ THỐNG

### 4.1. Sơ đồ kiến trúc

```
┌─────────────────────────────────────────────────────────────┐
│                    STM32F411 + FreeRTOS                     │
└─────────────────────────────────────────────────────────────┘

┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│ SensorTask     │  │ UartTask       │  │ LedTask        │
│ Priority: High │  │ Priority:      │  │ Priority:      │
│                │  │ AboveNormal    │  │ BelowNormal    │
│ - Read BME280  │  │                │  │                │
│ - Read CCS811  │──▶│ - Get from    │  │ - Blink LED    │
│ - Read BH1750  │  │   Queue        │  │ - Show status  │
│ - Read Soil    │  │ - Format JSON  │  │                │
│ - Put to Queue │  │ - Send UART    │  │                │
└────────────────┘  └────────────────┘  └────────────────┘
        │                                        
        │ sensorDataQueue                        
        ▼                                        
┌────────────────┐  ┌────────────────┐           
│ OledTask       │  │ MonitorTask    │           
│ Priority:      │  │ Priority: Low  │           
│ Normal         │  │                │           
│                │  │ - Heartbeat    │           
│ - Display data │  │ - Check status │           
│ - Page switch  │  │ - Error count  │           
└────────────────┘  └────────────────┘           

┌─────────────────────────────────────────┐
│        Shared Resources (Mutexes)       │
├─────────────────────────────────────────┤
│ - spiMutex     (BME280 SPI access)     │
│ - uartMutex    (UART TX protection)    │
│ - oledMutex    (I2C OLED protection)   │
└─────────────────────────────────────────┘
```

### 4.2. Luồng dữ liệu

```
SENSORS ──▶ SensorTask ──▶ sensorDataQueue ──▶ UartTask ──▶ UART (PC)
   │                              │
   │                              └──▶ OledTask ──▶ OLED Display
   │
   └──▶ MonitorTask ──▶ System Status
```

---

## 5. TASKS (NHIỆM VỤ)

### 5.1. Khái niệm Task

**Task** là một thread/luồng thực thi độc lập trong RTOS. Mỗi task:
- Có stack riêng (lưu local variables, return addresses)
- Có priority riêng
- Chạy như một vòng lặp vô hạn `for(;;)`
- Được scheduler quản lý và chuyển đổi

### 5.2. Tạo Task trong CMSIS-RTOS v2

#### **Cấu trúc cơ bản:**

```c
// 1. Khai báo handle (con trỏ) để quản lý task
osThreadId_t sensorTaskHandle;

// 2. Định nghĩa attributes (thuộc tính) của task
const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorTask",                    // Tên task (debug)
  .stack_size = 512 * 4,                  // Stack size in bytes
  .priority = (osPriority_t) osPriorityHigh, // Mức ưu tiên
};

// 3. Hàm task (chạy vô hạn)
void SensorReadingTask(void *argument)
{
  // Khởi tạo một lần
  InitializeSensors();
  
  // Vòng lặp vô hạn
  for(;;)
  {
    // Làm công việc
    ReadAllSensorData(&sensorData);
    
    // Gửi dữ liệu vào queue
    osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
    
    // Delay để nhường CPU cho tasks khác
    osDelay(3000); // 3 seconds
  }
}

// 4. Tạo task trong main()
int main(void)
{
  // ... HAL Init ...
  
  osKernelInitialize(); // Khởi tạo kernel
  
  // Tạo task
  sensorTaskHandle = osThreadNew(SensorReadingTask, NULL, &sensorTask_attributes);
  
  if (sensorTaskHandle == NULL) {
    Error_Handler(); // Xử lý lỗi nếu tạo task thất bại
  }
  
  osKernelStart(); // Khởi động scheduler
  
  // Không bao giờ đến đây
  while(1);
}
```

### 5.3. Các Task trong Project

Project này có **6 tasks**:

#### **Task 1: SensorReadingTask**
```c
Priority: osPriorityHigh (cao nhất)
Stack: 512 words (2048 bytes)
Period: 3000 ms

Chức năng:
- Đọc dữ liệu từ 4 sensors (BME280, CCS811, BH1750, Soil Moisture)
- Sử dụng Mutex để bảo vệ SPI bus
- Đưa dữ liệu vào sensorDataQueue
- Kiểm tra độ ẩm đất, gửi cảnh báo nếu cần
- Cập nhật LED status

Code đơn giản hóa:
void SensorReadingTask(void *argument)
{
  SensorData_t sensorData;
  InitializeSensors();
  
  for(;;)
  {
    // Acquire SPI mutex
    if (osMutexAcquire(spiMutexHandle, 1000) == osOK) {
      ReadAllSensorData(&sensorData);
      osMutexRelease(spiMutexHandle);
      
      // Gửi vào queue
      osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
    }
    
    osDelay(3000); // Đọc mỗi 3 giây
  }
}
```

**Tại sao priority cao?**
- Sensors cần được đọc đúng lúc (real-time)
- Không được miss timing để đảm bảo data accuracy

#### **Task 2: UartCommunicationTask**
```c
Priority: osPriorityAboveNormal
Stack: 768 words (3072 bytes)
Period: Non-blocking (50 ms check)

Chức năng:
- Nhận data từ sensorDataQueue
- Format data (JSON/CSV/Plain text)
- Gửi qua UART đến PC
- Sử dụng UART Mutex để tránh conflict

Code đơn giản hóa:
void UartCommunicationTask(void *argument)
{
  SensorData_t receivedData;
  
  for(;;)
  {
    // Lấy data từ queue (timeout 50ms)
    if (osMessageQueueGet(sensorDataQueueHandle, &receivedData, NULL, 50) == osOK) 
    {
      // Acquire UART mutex
      if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
        FormatSensorData(&receivedData, buffer, sizeof(buffer));
        HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);
        osMutexRelease(uartMutexHandle);
      }
    }
    
    osDelay(50);
  }
}
```

**Tại sao stack lớn?**
- Cần buffer lớn để format JSON string (512 bytes)
- sprintf/snprintf tốn stack

#### **Task 3: LedStatusTask**
```c
Priority: osPriorityBelowNormal (thấp)
Stack: 256 words (1024 bytes)
Period: 500 ms

Chức năng:
- Kiểm tra trạng thái sensors
- Nhấp nháy LED nếu có lỗi
- Bật LED liên tục nếu OK

Code:
void LedStatusTask(void *argument)
{
  for(;;)
  {
    bool anySensorConnected = systemStatus.bme280_connected || 
                             systemStatus.ccs811_connected;
    
    if (anySensorConnected) {
      LED_On(); // Solid ON
    } else {
      LED_Toggle(); // Blink
    }
    
    osDelay(500);
  }
}
```

**Tại sao priority thấp?**
- LED chỉ là visual feedback, không quan trọng
- Có thể bị chiếm quyền bởi tasks quan trọng hơn

#### **Task 4: SystemMonitorTask**
```c
Priority: osPriorityLow
Stack: 512 words
Period: 60000 ms (1 phút)

Chức năng:
- Kiểm tra kết nối sensors
- Cập nhật uptime
- Gửi heartbeat message
- Đếm số lỗi

Code:
void SystemMonitorTask(void *argument)
{
  for(;;)
  {
    UpdateSystemStatus(); // Kiểm tra sensors
    
    // Gửi heartbeat
    snprintf(heartbeatMsg.data, sizeof(heartbeatMsg.data),
             "{\"type\":\"HEARTBEAT\",\"uptime\":%lu,...}",
             systemStatus.uptime_seconds);
    
    osMessageQueuePut(uartTxQueueHandle, &heartbeatMsg, 0, 100);
    
    osDelay(60000); // Mỗi phút
  }
}
```

#### **Task 5: OledDisplayTask**
```c
Priority: osPriorityNormal
Stack: 512 words
Chức năng:
- Hiển thị data lên màn hình OLED
- Tự động chuyển trang mỗi 4 giây
- Sử dụng OLED Mutex để bảo vệ I2C bus
```

#### **Task 6: DefaultTask**
```c
Priority: osPriorityNormal
Stack: 128 words (minimal)

Chức năng:
- Task mặc định do STM32CubeMX tạo
- Không làm gì, chỉ delay(1)
- Có thể xóa hoặc sử dụng cho mục đích khác
```

### 5.4. Task States (Trạng thái Task)

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

**Running**: Task đang chạy trên CPU  
**Ready**: Task sẵn sàng chạy, chờ scheduler  
**Blocked**: Task đang chờ (delay, queue, mutex)  
**Suspended**: Task bị tạm dừng (không dùng trong project này)

---

## 6. QUEUES (HÀNG ĐỢI)

### 6.1. Khái niệm Queue

**Queue** là cấu trúc dữ liệu FIFO (First-In-First-Out) để truyền dữ liệu giữa các tasks một cách thread-safe.

**Ưu điểm:**
- An toàn thread (không cần mutex riêng)
- Tách biệt producer và consumer
- Buffering data

### 6.2. Tạo Queue

```c
// 1. Khai báo handle
osMessageQueueId_t sensorDataQueueHandle;

// 2. Định nghĩa cấu trúc dữ liệu
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
    // ... các field khác
} SensorData_t;

// 3. Tạo queue trong main()
sensorDataQueueHandle = osMessageQueueNew(
    5,                      // Queue capacity (5 messages)
    sizeof(SensorData_t),   // Size của mỗi message
    NULL                    // Attributes (NULL = default)
);

if (sensorDataQueueHandle == NULL) {
    Error_Handler();
}
```

### 6.3. Sử dụng Queue

#### **Producer (SensorTask) - Gửi data vào queue:**
```c
SensorData_t sensorData;
ReadAllSensorData(&sensorData);

// Put vào queue
osStatus_t status = osMessageQueuePut(
    sensorDataQueueHandle,  // Queue handle
    &sensorData,            // Pointer to data
    0,                      // Priority (0 = normal)
    100                     // Timeout (100 ticks = 100 ms)
);

if (status != osOK) {
    // Queue đầy hoặc timeout
    systemStatus.sensor_errors++;
}
```

#### **Consumer (UartTask) - Lấy data từ queue:**
```c
SensorData_t receivedData;

// Get từ queue
osStatus_t status = osMessageQueueGet(
    sensorDataQueueHandle,  // Queue handle
    &receivedData,          // Buffer để nhận data
    NULL,                   // Priority output (không quan tâm)
    50                      // Timeout (50 ms)
);

if (status == osOK) {
    // Có data mới
    FormatSensorData(&receivedData, buffer, sizeof(buffer));
    HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);
}
```

### 6.4. Queues trong Project

#### **Queue 1: sensorDataQueue**
```c
Type: SensorData_t (64 bytes/message)
Capacity: 5 messages
Total Size: 5 × 64 = 320 bytes

Luồng dữ liệu:
SensorTask ──▶ sensorDataQueue ──▶ UartTask
```

**Tại sao capacity = 5?**
- SensorTask gửi mỗi 3 giây
- UartTask nhận và xử lý nhanh (~100 ms)
- 5 messages = buffer 15 giây data → đủ dự phòng

#### **Queue 2: uartTxQueue**
```c
Type: UartMessage_t (258 bytes/message)
Capacity: 10 messages
Total Size: 10 × 258 = 2580 bytes

Cấu trúc:
typedef struct {
    char data[256];      // Message content
    uint16_t length;     // Message length
} UartMessage_t;

Luồng dữ liệu:
MonitorTask ──▶ uartTxQueue ──▶ UartTask
SensorTask  ──▶ (irrigation alerts)
```

**Tại sao cần queue riêng?**
- MonitorTask gửi heartbeat messages (không phải sensor data)
- SensorTask gửi irrigation alerts
- Tách biệt luồng control messages và sensor data

---

## 7. MUTEXES (KHÓA TƯƠNG HỖ)

### 7.1. Khái niệm Mutex

**Mutex** (Mutual Exclusion) là cơ chế đồng bộ để bảo vệ tài nguyên chia sẻ (shared resources).

**Khi nào cần Mutex?**
- Nhiều tasks truy cập cùng một hardware (SPI, UART, I2C)
- Nhiều tasks đọc/ghi cùng một biến global
- Tránh race condition

**Nguyên tắc:**
```
Task A                     Shared Resource                Task B
  │                              │                           │
  ├──── Acquire Mutex ───────────┤                           │
  │     (Lock)                   │                           │
  │                              │                           │
  ├──── Access SPI Bus ─────────▶│                           │
  │                              │                           │
  │                              │                           │
  ├──── Release Mutex ───────────┤                           │
  │     (Unlock)                 │                           │
  │                              │                           │
  │                              ├─── Acquire Mutex ─────────┤
  │                              │   (Task B đã chờ)         │
  │                              │                           │
  │                              │◀──── Access SPI Bus ──────┤
  │                              │                           │
  │                              ├─── Release Mutex ─────────┤
```

### 7.2. Tạo và sử dụng Mutex

```c
// 1. Khai báo handle
osMutexId_t spiMutexHandle;

// 2. Tạo mutex trong main()
spiMutexHandle = osMutexNew(NULL); // NULL = default attributes

if (spiMutexHandle == NULL) {
    Error_Handler();
}

// 3. Sử dụng trong Task
void SensorReadingTask(void *argument)
{
  for(;;)
  {
    // Acquire mutex (chờ tối đa 1000 ms)
    if (osMutexAcquire(spiMutexHandle, 1000) == osOK) 
    {
      // CRITICAL SECTION - Chỉ 1 task được vào
      ReadAllSensorData(&sensorData); // Truy cập SPI bus
      
      // Release mutex
      osMutexRelease(spiMutexHandle);
    }
    else 
    {
      // Timeout - không acquire được mutex
      systemStatus.sensor_errors++;
    }
    
    osDelay(3000);
  }
}
```

### 7.3. Mutexes trong Project

#### **Mutex 1: spiMutex**
```c
Mục đích: Bảo vệ SPI3 bus (BME280 sensor)
Shared Resource: SPI3 peripheral
Tasks sử dụng: SensorTask

Tại sao cần:
- BME280 sử dụng SPI3
- Dù chỉ 1 task hiện tại, nhưng thiết kế cho future-proof
- Tránh lỗi nếu thêm tasks khác cần SPI sau này
```

#### **Mutex 2: uartMutex**
```c
Mục đích: Bảo vệ UART2 peripheral
Shared Resource: UART2 (TX)
Tasks sử dụng: UartTask

Lý do:
- UartTask nhận data từ 2 queues (sensorDataQueue và uartTxQueue)
- Tránh 2 nguồn data gửi đồng thời gây lỗi UART
```

#### **Mutex 3: oledMutex**
```c
Mục đích: Bảo vệ I2C3 bus (OLED display)
Shared Resource: I2C3 peripheral và SSD1306 display
Tasks sử dụng: SensorTask, OledTask

Tại sao:
- SensorTask cập nhật OLED mỗi khi đọc sensor
- OledTask có thể reinitialize OLED nếu disconnected
- I2C không thread-safe, cần mutex
```

### 7.4. Mutex vs Semaphore

**Mutex:**
- Chỉ task nào acquire mới được release (ownership)
- Hỗ trợ priority inheritance (tránh priority inversion)
- Dùng cho mutual exclusion

**Semaphore:**
- Bất kỳ task nào cũng có thể signal/wait
- Không có ownership
- Dùng cho synchronization và counting

**Project này chỉ dùng Mutex vì:**
- Cần bảo vệ resources (SPI, UART, I2C)
- Cần priority inheritance để tránh deadlock

---

## 8. ĐỒNG BỘ HÓA VÀ CHIA SẺ TÀI NGUYÊN

### 8.1. Producer-Consumer Pattern

```c
// Producer: SensorTask
void SensorReadingTask(void *argument)
{
  for(;;)
  {
    // 1. Acquire mutex để đọc sensor
    osMutexAcquire(spiMutexHandle, 1000);
    
    // 2. Đọc data
    ReadAllSensorData(&sensorData);
    
    // 3. Release mutex
    osMutexRelease(spiMutexHandle);
    
    // 4. Put vào queue (không cần mutex vì queue đã thread-safe)
    osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
    
    osDelay(3000);
  }
}

// Consumer: UartTask
void UartCommunicationTask(void *argument)
{
  for(;;)
  {
    // 1. Get từ queue (blocking)
    if (osMessageQueueGet(sensorDataQueueHandle, &receivedData, NULL, 50) == osOK)
    {
      // 2. Acquire mutex để gửi UART
      osMutexAcquire(uartMutexHandle, 1000);
      
      // 3. Format và gửi
      FormatSensorData(&receivedData, buffer, sizeof(buffer));
      HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);
      
      // 4. Release mutex
      osMutexRelease(uartMutexHandle);
    }
    
    osDelay(50);
  }
}
```

### 8.2. Race Condition và cách tránh

**Race Condition** xảy ra khi 2+ tasks truy cập cùng 1 biến global không được bảo vệ.

**Ví dụ lỗi:**
```c
// Global variable (KHÔNG AN TOÀN)
uint32_t errorCounter = 0;

// Task A
void TaskA(void *argument) {
  for(;;) {
    errorCounter++; // RACE CONDITION!
    osDelay(100);
  }
}

// Task B
void TaskB(void *argument) {
  for(;;) {
    errorCounter++; // RACE CONDITION!
    osDelay(200);
  }
}

// Vấn đề: errorCounter++ không atomic (3 bước: read-modify-write)
// Task A và B có thể đọc cùng lúc → mất data
```

**Giải pháp 1: Mutex**
```c
osMutexId_t errorMutexHandle;

void TaskA(void *argument) {
  for(;;) {
    osMutexAcquire(errorMutexHandle, osWaitForever);
    errorCounter++;
    osMutexRelease(errorMutexHandle);
    osDelay(100);
  }
}
```

**Giải pháp 2: Atomic operations (nếu compiler hỗ trợ)**
```c
#include <stdatomic.h>
atomic_uint errorCounter = 0;

void TaskA(void *argument) {
  for(;;) {
    atomic_fetch_add(&errorCounter, 1); // Thread-safe
    osDelay(100);
  }
}
```

**Trong project này:**
```c
// SystemStatus_t được cập nhật chỉ bởi 1 task (MonitorTask)
// → Không cần mutex

// systemStatus được đọc bởi nhiều tasks
// → An toàn vì chỉ read (không modify)

typedef struct {
    bool bme280_connected;
    bool ccs811_connected;
    // ...
    uint32_t sensor_errors;
    uint32_t uptime_seconds;
} SystemStatus_t;

SystemStatus_t systemStatus = {false, false, ...};
```

### 8.3. Deadlock và cách tránh

**Deadlock** xảy ra khi 2 tasks chờ nhau giải phóng mutex.

**Ví dụ deadlock:**
```c
// Task A
osMutexAcquire(mutexSPI, osWaitForever);
osMutexAcquire(mutexI2C, osWaitForever); // Chờ Task B
osMutexRelease(mutexI2C);
osMutexRelease(mutexSPI);

// Task B
osMutexAcquire(mutexI2C, osWaitForever);
osMutexAcquire(mutexSPI, osWaitForever); // Chờ Task A → DEADLOCK!
osMutexRelease(mutexSPI);
osMutexRelease(mutexI2C);
```

**Cách tránh:**
1. **Lock ordering**: Luôn acquire mutex theo thứ tự cố định
2. **Timeout**: Dùng timeout thay vì osWaitForever
3. **Single mutex**: Mỗi task chỉ acquire 1 mutex tại 1 thời điểm

**Trong project này:**
```c
// Mỗi task chỉ acquire 1 mutex
// → Không có deadlock

// SensorTask: chỉ dùng spiMutex
// UartTask: chỉ dùng uartMutex
// OledTask: chỉ dùng oledMutex
```

---

## 9. QUẢN LÝ BỘ NHỚ

### 9.1. Heap Memory

FreeRTOS sử dụng heap để cấp phát:
- Task control blocks (TCB)
- Task stacks
- Queues
- Mutexes
- Timers

**Heap Implementation:**
```c
// FreeRTOSConfig.h
#define USE_FreeRTOS_HEAP_4

// heap_4.c:
// - Cho phép malloc() và free()
// - Chống phân mảnh bộ nhớ
// - Phù hợp cho ứng dụng tạo/xóa tasks động
```

**Kiểm tra heap còn trống:**
```c
size_t freeHeap = xPortGetFreeHeapSize();
printf("Free heap: %u bytes\r\n", freeHeap);

// Hoặc dùng FreeRTOS trace tools
```

### 9.2. Stack Overflow Detection

**Nguyên nhân stack overflow:**
- Local variables quá lớn
- Đệ quy sâu
- sprintf() với buffer lớn
- Large struct trên stack

**Phát hiện:**
```c
// FreeRTOSConfig.h
#define configCHECK_FOR_STACK_OVERFLOW  2

// freertos.c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // Task bị stack overflow → gọi hook này
    printf("Stack overflow in task: %s\r\n", pcTaskName);
    Error_Handler();
}
```

**Kiểm tra stack usage:**
```c
UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(sensorTaskHandle);
printf("Sensor Task stack free: %u words\r\n", stackHighWaterMark);

// High water mark = số words chưa dùng
// Nếu < 50 words → cần tăng stack size
```

### 9.3. Tối ưu bộ nhớ

**1. Dùng static allocation khi có thể:**
```c
// Dynamic (dùng heap)
sensorTaskHandle = osThreadNew(SensorReadingTask, NULL, &attributes);

// Static (không dùng heap)
StaticTask_t sensorTaskBuffer;
uint32_t sensorTaskStack[512];

osThreadAttr_t attributes = {
  .name = "SensorTask",
  .cb_mem = &sensorTaskBuffer,
  .cb_size = sizeof(sensorTaskBuffer),
  .stack_mem = &sensorTaskStack,
  .stack_size = sizeof(sensorTaskStack),
};

sensorTaskHandle = osThreadNew(SensorReadingTask, NULL, &attributes);
```

**2. Giảm stack size nếu có thể:**
```c
// Kiểm tra xem task dùng bao nhiêu stack thực tế
UBaseType_t used = uxTaskGetStackHighWaterMark(ledTaskHandle);

// Nếu LED task chỉ dùng 100 words → giảm từ 256 xuống 128
```

**3. Tránh large local variables:**
```c
// XẤU
void TaskFunction(void *argument) {
  char bigBuffer[2048]; // 2KB trên stack!
  // ...
}

// TỐT
char bigBuffer[2048]; // Global hoặc static
void TaskFunction(void *argument) {
  // Hoặc dùng malloc() nếu cần dynamic
  char *buffer = pvPortMalloc(2048);
  // ...
  vPortFree(buffer);
}
```

---

## 10. XỬ LÝ LỖI VÀ DEBUG

### 10.1. Error Handling

**1. Kiểm tra return status:**
```c
osStatus_t status = osMessageQueuePut(queueHandle, &data, 0, 100);

if (status != osOK) {
  switch (status) {
    case osErrorTimeout:
      // Queue đầy, timeout
      systemStatus.sensor_errors++;
      break;
    case osErrorParameter:
      // Tham số không hợp lệ
      Error_Handler();
      break;
    default:
      // Lỗi khác
      break;
  }
}
```

**2. Malloc Failed Hook:**
```c
// freertos.c
void vApplicationMallocFailedHook(void)
{
  // Heap hết bộ nhớ
  // → Tăng configTOTAL_HEAP_SIZE
  __disable_irq();
  while(1); // Halt
}
```

**3. Assert:**
```c
// FreeRTOSConfig.h
#define configASSERT(x) if ((x) == 0) { \
    taskDISABLE_INTERRUPTS(); \
    for( ;; ); \
}

// Sử dụng
configASSERT(sensorTaskHandle != NULL);
```

### 10.2. Debugging với UART

```c
// Gửi debug message
void DebugPrint(const char *format, ...)
{
  char buffer[128];
  va_list args;
  
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  osMutexAcquire(uartMutexHandle, osWaitForever);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
  osMutexRelease(uartMutexHandle);
}

// Task
void SensorReadingTask(void *argument)
{
  DebugPrint("[SensorTask] Started\r\n");
  
  for(;;)
  {
    if (ReadAllSensorData(&data) != HAL_OK) {
      DebugPrint("[ERROR] Sensor read failed\r\n");
    }
    osDelay(3000);
  }
}
```

### 10.3. Runtime Statistics

```c
// FreeRTOSConfig.h
#define configGENERATE_RUN_TIME_STATS       1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

// Lấy CPU usage
char statsBuffer[512];
vTaskGetRunTimeStats(statsBuffer);
printf("%s\r\n", statsBuffer);

// Output:
// Task            Abs Time        % Time
// SensorTask      1234567         45%
// UartTask        567890          20%
// LedTask         123456          5%
// ...
```

---

## 11. BEST PRACTICES

### 11.1. Task Design

**1. Mỗi task một trách nhiệm:**
```c
// TỐT: Tách biệt rõ ràng
SensorTask    → Đọc sensor
UartTask      → Gửi UART
LedTask       → Điều khiển LED

// XẤU: Task làm quá nhiều việc
SuperTask     → Đọc sensor + Gửi UART + LED + OLED + ...
```

**2. Luôn có delay trong task loop:**
```c
// XẤU: Không có delay → 100% CPU
void TaskFunction(void *argument) {
  for(;;) {
    DoWork();
    // Không có delay!
  }
}

// TỐT: Có delay để nhường CPU
void TaskFunction(void *argument) {
  for(;;) {
    DoWork();
    osDelay(100); // Nhường CPU
  }
}
```

**3. Sử dụng blocking operations:**
```c
// TỐT: Blocking - task ngủ khi chờ
osStatus_t status = osMessageQueueGet(queueHandle, &data, NULL, osWaitForever);

// XẤU: Polling - lãng phí CPU
while (osMessageQueueGet(queueHandle, &data, NULL, 0) != osOK) {
  // Busy wait - 100% CPU!
}
```

### 11.2. Priority Assignment

**Nguyên tắc:**
```
Priority cao    → Real-time critical tasks
Priority trung  → Normal processing
Priority thấp   → Background tasks
```

**Trong project này:**
```
osPriorityHigh         → SensorTask (đọc sensor real-time)
osPriorityAboveNormal  → UartTask (giao tiếp quan trọng)
osPriorityNormal       → OledTask, DefaultTask
osPriorityBelowNormal  → LedTask (visual feedback)
osPriorityLow          → MonitorTask (heartbeat)
```

**Tránh priority inversion:**
```c
// Nếu low-priority task giữ mutex mà high-priority task cần
// → FreeRTOS tự động tăng priority tạm thời (priority inheritance)
// → Không cần làm gì đặc biệt
```

### 11.3. Queue Sizing

**Công thức:**
```
Queue Size = (Producer Rate / Consumer Rate) × Safety Factor

Ví dụ:
Producer (SensorTask): 1 message / 3s
Consumer (UartTask):   xử lý trong ~100ms

Tỷ lệ: 3000ms / 100ms = 30 lần nhanh hơn
→ Queue size = 1 message là đủ

Safety factor = 5× → Queue size = 5 messages
```

### 11.4. Mutex Best Practices

**1. Giữ mutex trong thời gian ngắn nhất:**
```c
// XẤU: Giữ mutex quá lâu
osMutexAcquire(uartMutex, osWaitForever);
FormatData();        // 50 ms
TransmitUART();      // 100 ms
ProcessResult();     // 30 ms
osMutexRelease(uartMutex); // Tổng 180 ms!

// TỐT: Chỉ lock khi cần
FormatData();        // Không cần mutex
osMutexAcquire(uartMutex, osWaitForever);
TransmitUART();      // 100 ms
osMutexRelease(uartMutex);
ProcessResult();     // Không cần mutex
```

**2. Dùng timeout:**
```c
// XẤU: Chờ mãi mãi
osMutexAcquire(mutex, osWaitForever);

// TỐT: Có timeout
if (osMutexAcquire(mutex, 1000) == osOK) {
  // Critical section
  osMutexRelease(mutex);
} else {
  // Timeout - xử lý lỗi
  LogError("Mutex timeout");
}
```

**3. Luôn release mutex:**
```c
// TỐT: Luôn release ngay cả khi có lỗi
if (osMutexAcquire(mutex, 1000) == osOK) {
  if (DoWork() != SUCCESS) {
    osMutexRelease(mutex); // Release trước khi return
    return ERROR;
  }
  osMutexRelease(mutex);
}
```

### 11.5. Naming Conventions

```c
// Task handles
osThreadId_t sensorTaskHandle;
osThreadId_t uartTaskHandle;

// Task attributes
const osThreadAttr_t sensorTask_attributes = {...};

// Task functions
void SensorReadingTask(void *argument);

// Queue handles
osMessageQueueId_t sensorDataQueueHandle;

// Mutex handles
osMutexId_t spiMutexHandle;

// Global variables
SystemStatus_t systemStatus;

// #defines
#define SENSOR_TASK_STACK_SIZE    512
#define SENSOR_READ_PERIOD_MS     3000
```

---

## 12. KẾT LUẬN

### 12.1. Tóm tắt kiến trúc RTOS trong project

```
┌─────────────────────────────────────────────────────────────┐
│  STM32F411 + FreeRTOS Sensor Monitoring System             │
├─────────────────────────────────────────────────────────────┤
│  6 Tasks:                                                   │
│  - SensorTask (High priority): Đọc 4 sensors mỗi 3s       │
│  - UartTask (AboveNormal): Gửi data qua UART              │
│  - LedTask (BelowNormal): Hiển thị status                 │
│  - OledTask (Normal): Cập nhật màn hình OLED              │
│  - MonitorTask (Low): Heartbeat mỗi phút                   │
│  - DefaultTask (Normal): Idle                              │
│                                                             │
│  2 Queues:                                                  │
│  - sensorDataQueue: SensorTask → UartTask/OledTask        │
│  - uartTxQueue: MonitorTask → UartTask                     │
│                                                             │
│  3 Mutexes:                                                 │
│  - spiMutex: Bảo vệ SPI3 (BME280)                         │
│  - uartMutex: Bảo vệ UART2                                │
│  - oledMutex: Bảo vệ I2C3 (OLED)                          │
│                                                             │
│  Heap: 15 KB (heap_4)                                       │
│  Tick: 1 ms (1000 Hz)                                       │
└─────────────────────────────────────────────────────────────┘
```

### 12.2. Lợi ích của RTOS trong project này

**1. Modularity (Tính module):**
- Mỗi chức năng là 1 task riêng biệt
- Dễ thêm/bớt features
- Code rõ ràng, dễ maintain

**2. Real-time Performance:**
- SensorTask đọc sensors đúng thời gian (mỗi 3s)
- Không bị delay bởi UART hoặc OLED
- LED phản hồi ngay lập tức

**3. Resource Sharing:**
- Mutexes bảo vệ SPI, UART, I2C
- Tránh race conditions
- Thread-safe

**4. Scalability:**
- Dễ thêm tasks mới (ví dụ: WiFi task, SD card task)
- Dễ tăng số sensors
- Dễ mở rộng chức năng

### 12.3. So sánh RTOS vs Bare-Metal

| Aspect | RTOS | Bare-Metal |
|--------|------|------------|
| Code structure | Tasks riêng biệt | Một vòng lặp while(1) lớn |
| Timing | osDelay() chính xác | HAL_Delay() blocking |
| Multitasking | Tự động bởi scheduler | Phải code thủ công (state machine) |
| Resource sharing | Mutexes tự động | Phải disable interrupts |
| Memory | Cần ~15 KB heap | Ít hơn (~5 KB) |
| Complexity | Cao hơn (learning curve) | Đơn giản hơn |
| Scalability | Tốt (thêm tasks dễ) | Khó (code càng phức tạp) |

**Khi nào dùng RTOS?**
- Hệ thống phức tạp (nhiều sensors, nhiều chức năng)
- Cần real-time response
- Nhiều tasks chạy "đồng thời"
- Team lớn (modularity tốt)

**Khi nào dùng Bare-Metal?**
- Ứng dụng đơn giản (vài sensors)
- RAM/Flash hạn chế
- Không cần multitasking
- Yêu cầu deterministic timing tuyệt đối

### 12.4. Checklist cho RTOS project

**Thiết kế:**
- [ ] Xác định các tasks cần thiết
- [ ] Gán priorities phù hợp
- [ ] Tính toán stack size cho mỗi task
- [ ] Xác định shared resources và cần mutexes
- [ ] Thiết kế inter-task communication (queues)

**Implementation:**
- [ ] Cấu hình FreeRTOSConfig.h đúng
- [ ] Tạo tasks với attributes rõ ràng
- [ ] Tạo queues với size phù hợp
- [ ] Tạo mutexes cho shared resources
- [ ] Luôn có delay trong task loops
- [ ] Kiểm tra return status của RTOS APIs

**Testing:**
- [ ] Kiểm tra heap usage (xPortGetFreeHeapSize)
- [ ] Kiểm tra stack overflow (uxTaskGetStackHighWaterMark)
- [ ] Test deadlock scenarios
- [ ] Test với stress load
- [ ] Measure CPU usage

**Debug:**
- [ ] Enable stack overflow detection
- [ ] Enable malloc failed hook
- [ ] Add UART debug prints
- [ ] Use runtime statistics

### 12.5. Tài liệu tham khảo

1. **FreeRTOS Official Documentation**
   - https://www.freertos.org/Documentation/RTOS_book.html
   
2. **CMSIS-RTOS v2 API Reference**
   - https://arm-software.github.io/CMSIS_5/RTOS2/html/index.html

3. **STM32 RTOS Guide**
   - AN4044: Using FreeRTOS with STM32

4. **Book: Mastering the FreeRTOS Real Time Kernel**
   - Free PDF from FreeRTOS website

---

## PHỤ LỤC

### A. Các hàm CMSIS-RTOS v2 thường dùng

#### Threads (Tasks)
```c
osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr);
osStatus_t osThreadTerminate(osThreadId_t thread_id);
osStatus_t osDelay(uint32_t ticks);
osStatus_t osThreadYield(void);
```

#### Queues
```c
osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout);
```

#### Mutexes
```c
osMutexId_t osMutexNew(const osMutexAttr_t *attr);
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex_id);
```

#### Kernel
```c
osStatus_t osKernelInitialize(void);
osStatus_t osKernelStart(void);
uint32_t osKernelGetTickCount(void);
```

### B. Error Codes

```c
typedef enum {
  osOK                    =  0,  // Success
  osError                 = -1,  // Unspecified error
  osErrorTimeout          = -2,  // Timeout
  osErrorResource         = -3,  // Resource not available
  osErrorParameter        = -4,  // Invalid parameter
  osErrorNoMemory         = -5,  // Out of memory
  osErrorISR              = -6,  // Not allowed in ISR context
} osStatus_t;
```

### C. Công thức tính toán

**1. Heap Size:**
```
Heap = Σ(Task TCB + Task Stack) + Σ(Queue Sizes) + Σ(Mutexes) + Overhead

TCB ≈ 200 bytes/task
Mutex ≈ 100 bytes
Overhead ≈ 10% of total
```

**2. CPU Utilization:**
```
CPU% = (Task Runtime / Total Runtime) × 100%
```

**3. Queue Depth:**
```
Queue Depth = (Producer Period / Consumer Period) × Safety Factor

Safety Factor thường = 2 đến 5
```

---

**HẾT BÁO CÁO**

---

*Báo cáo này cung cấp hướng dẫn chi tiết về cách viết chương trình RTOS trên STM32 với FreeRTOS. Hy vọng nó giúp ích cho việc học tập và phát triển các dự án embedded RTOS trong tương lai.*
