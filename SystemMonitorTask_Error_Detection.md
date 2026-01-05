# CÁCH SYSTEMMONITORTASK PHÁT HIỆN LỖI CẢM BIẾN

## Tổng quan
SystemMonitorTask trong dự án STM32F411CEU6 sử dụng cơ chế **theo dõi lỗi phi tập trung với báo cáo tập trung** để phát hiện và báo cáo lỗi cảm biến. Hệ thống này hoạt động dựa trên việc chia sẻ biến trạng thái toàn cục giữa các task.

## 1. Cấu trúc dữ liệu theo dõi lỗi

### SystemStatus_t Structure (dòng 55-70 trong main.c)
```c
typedef struct {
    uint32_t uptime_seconds;
    uint32_t free_heap_size;
    uint8_t sensor_errors;      // Bộ đếm lỗi cảm biến
    uint8_t uart_errors;        // Bộ đếm lỗi UART
    bool bme280_connected;      // Trạng thái kết nối BME280
    bool ccs811_connected;      // Trạng thái kết nối CCS811
    bool bh1750_connected;      // Trạng thái kết nối BH1750
    bool soil_sensor_connected; // Trạng thái kết nối cảm biến đất
} SystemStatus_t;

// Biến toàn cục được chia sẻ giữa các task
SystemStatus_t systemStatus = {0};
```

## 2. Cơ chế phát hiện lỗi

### 2.1 Phát hiện lỗi bởi SensorReadingTask

#### A. Lỗi Mutex Timeout (dòng 910-925)
```c
// Trong SensorReadingTask - Khi không thể lấy SPI mutex
if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Đọc cảm biến thành công
} else {
    systemStatus.sensor_errors++;  // Tăng bộ đếm lỗi
    printf("Error: Cannot acquire SPI mutex\r\n");
}
```

#### B. Lỗi Queue gửi dữ liệu (dòng 945-965)
```c
// Khi không thể gửi dữ liệu vào queue
if (xQueueSend(sensorDataQueue, &sensorData, pdMS_TO_TICKS(50)) != pdTRUE) {
    systemStatus.sensor_errors++;  // Tăng bộ đếm lỗi
    printf("Error: Failed to send sensor data to queue\r\n");
}
```

### 2.2 Phát hiện lỗi bởi UartCommunicationTask

#### A. Lỗi UART Transmission
```c
// Khi không thể truyền dữ liệu qua UART
if (HAL_UART_Transmit(&huart2, buffer, length, 1000) != HAL_OK) {
    systemStatus.uart_errors++;    // Tăng bộ đếm lỗi UART
}
```

#### B. Lỗi UART Mutex Timeout
```c
// Khi không thể lấy UART mutex
if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    systemStatus.uart_errors++;    // Tăng bộ đếm lỗi UART
}
```

## 3. Hàm UpdateSystemStatus() - Kiểm tra kết nối cảm biến

### Vị trí: dòng 866-880 trong main.c
```c
void UpdateSystemStatus(void)
{
    // Cập nhật thời gian hoạt động
    systemStatus.uptime_seconds = xTaskGetTickCount() / configTICK_RATE_HZ;
    
    // Cập nhật bộ nhớ heap còn trống
    systemStatus.free_heap_size = xPortGetFreeHeapSize();
    
    // Kiểm tra trạng thái kết nối cảm biến
    // (Logic kiểm tra kết nối dựa trên phản hồi từ các cảm biến)
    systemStatus.bme280_connected = CheckBME280Connection();
    systemStatus.ccs811_connected = CheckCCS811Connection();
    systemStatus.bh1750_connected = CheckBH1750Connection();
    systemStatus.soil_sensor_connected = CheckSoilSensorConnection();
}
```

## 4. SystemMonitorTask - Thu thập và báo cáo lỗi

### 4.1 Chu kỳ hoạt động (dòng 1015-1035)
```c
void SystemMonitorTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(60000); // 60 giây
    
    for(;;)
    {
        // Cập nhật trạng thái hệ thống
        UpdateSystemStatus();
        
        // Tạo heartbeat message với thông tin lỗi
        char heartbeat[200];
        snprintf(heartbeat, sizeof(heartbeat),
            "HEARTBEAT: Uptime=%lus, Heap=%lu, "
            "SensorErr=%u, UartErr=%u, "
            "BME280=%s, CCS811=%s, BH1750=%s, Soil=%s\r\n",
            systemStatus.uptime_seconds,
            systemStatus.free_heap_size,
            systemStatus.sensor_errors,     // Báo cáo số lỗi cảm biến
            systemStatus.uart_errors,       // Báo cáo số lỗi UART
            systemStatus.bme280_connected ? "OK" : "ERR",
            systemStatus.ccs811_connected ? "OK" : "ERR",
            systemStatus.bh1750_connected ? "OK" : "ERR",
            systemStatus.soil_sensor_connected ? "OK" : "ERR");
        
        // Gửi heartbeat qua UART
        SendUartMessage(heartbeat);
        
        // Chờ đến chu kỳ tiếp theo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

## 5. Luồng hoạt động tổng thể

### Bước 1: Phát hiện lỗi
- **SensorReadingTask**: Phát hiện lỗi mutex timeout, queue failure
- **UartCommunicationTask**: Phát hiện lỗi truyền UART, mutex timeout
- **Các task khác**: Có thể phát hiện các loại lỗi khác

### Bước 2: Ghi nhận lỗi
- Mỗi task tăng bộ đếm lỗi tương ứng trong `systemStatus`
- `sensor_errors++` cho lỗi liên quan đến cảm biến
- `uart_errors++` cho lỗi liên quan đến UART

### Bước 3: Kiểm tra định kỳ
- **SystemMonitorTask** chạy mỗi 60 giây
- Gọi `UpdateSystemStatus()` để kiểm tra kết nối cảm biến
- Cập nhật trạng thái kết nối cho từng cảm biến

### Bước 4: Báo cáo lỗi
- Tạo heartbeat message chứa:
  * Số lượng lỗi cảm biến tích lũy
  * Số lượng lỗi UART tích lũy  
  * Trạng thái kết nối từng cảm biến (OK/ERR)
  * Thông tin hệ thống (uptime, heap)

## 6. Ưu điểm của cơ chế này

### 6.1 Phi tập trung (Decentralized Detection)
- Mỗi task tự phát hiện lỗi trong phạm vi hoạt động của mình
- Không cần task monitor can thiệp vào quá trình đọc cảm biến
- Giảm overhead và độ phức tạp

### 6.2 Tập trung báo cáo (Centralized Reporting)
- SystemMonitorTask thu thập tất cả thông tin lỗi
- Báo cáo thống nhất qua một kênh UART
- Dễ dàng theo dõi và debug

### 6.3 Theo dõi lâu dài
- Bộ đếm lỗi tích lũy cho phép đánh giá độ tin cậy
- Heartbeat định kỳ giúp phát hiện system hang
- Thông tin trạng thái chi tiết cho từng cảm biến

## 7. Ví dụ hoạt động thực tế

### Kịch bản: Cảm biến BME280 bị ngắt kết nối

1. **T=0s**: Hệ thống hoạt động bình thường
   - `sensor_errors = 0`
   - `bme280_connected = true`

2. **T=5s**: BME280 bị ngắt kết nối, SensorReadingTask không đọc được
   - SensorReadingTask: `systemStatus.sensor_errors++` (sensor_errors = 1)
   - Mutex timeout hoặc SPI communication failure

3. **T=10s**: Lỗi tiếp tục xảy ra
   - SensorReadingTask: `systemStatus.sensor_errors++` (sensor_errors = 2)

4. **T=60s**: SystemMonitorTask báo cáo
   - Gọi `UpdateSystemStatus()`
   - `CheckBME280Connection()` return false
   - `bme280_connected = false`
   - Heartbeat: "SensorErr=2, BME280=ERR"

## 8. Kết luận

SystemMonitorTask không trực tiếp "phát hiện" lỗi cảm biến, mà hoạt động như một **trung tâm thu thập và báo cáo** thông tin lỗi đã được các task khác phát hiện. Cơ chế này đảm bảo:

- **Hiệu quả**: Mỗi task chỉ lo nhiệm vụ chính của mình
- **Đáng tin cậy**: Theo dõi lỗi liên tục và tích lũy
- **Dễ bảo trì**: Tập trung báo cáo, dễ thêm loại lỗi mới
- **Thời gian thực**: Phát hiện lỗi ngay khi xảy ra, báo cáo định kỳ

Đây là một pattern phổ biến trong embedded systems để đảm bảo system reliability và fault tolerance.