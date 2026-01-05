# STM32F411CEU6 + BME280 + CCS811 + FreeRTOS Project

## Mô tả
Chương trình RTOS cho STM32F411CEU6 đọc dữ liệu từ cảm biến BME280 (nhiệt độ, độ ẩm, áp suất) và CCS811 (CO2, TVOC), với LED báo trạng thái và truyền dữ liệu qua UART lên Orange Pi 4A.

## Cấu hình chân (Pin Configuration)
```
├── PC13: GPIO_Output (LED) - Active LOW
├── PB3:  I2C2_SDA (BME280 + CCS811)
├── PB10: I2C2_SCL (BME280 + CCS811)
├── PA2:  USART2_TX (Communication to OPI4A)
└── PA3:  USART2_RX (Communication from OPI4A)
```

## Địa chỉ I2C
- **BME280**: 0x76 (primary) hoặc 0x77 (secondary)
- **CCS811**: 0x5A (primary) hoặc 0x5B (secondary)

## Hoạt động LED PC13
- **Sáng liên tục**: Ít nhất một cảm biến hoạt động bình thường
- **Nhấp nháy**: Không có cảm biến nào hoạt động

## FreeRTOS Tasks

### 1. SensorReadingTask (Priority: High)
- **Stack size**: 512 words
- **Chu kỳ**: 3 giây
- **Chức năng**:
  - Khởi tạo và đọc dữ liệu từ BME280 và CCS811
  - Sử dụng I2C mutex để đồng bộ hóa truy cập
  - Gửi dữ liệu vào sensor data queue
  - Tự động khởi tạo lại cảm biến khi lỗi

### 2. UartCommunicationTask (Priority: Above Normal)
- **Stack size**: 768 words
- **Chức năng**:
  - Nhận dữ liệu từ sensor queue và format theo JSON/CSV/Plain text
  - Truyền dữ liệu qua UART2 (115200 baud)
  - Xử lý lệnh từ Orange Pi 4A
  - Sử dụng UART mutex để đồng bộ hóa

### 3. LedStatusTask (Priority: Below Normal)
- **Stack size**: 256 words
- **Chu kỳ**: 500ms
- **Chức năng**:
  - Điều khiển LED PC13 dựa trên trạng thái cảm biến
  - LED sáng: có ít nhất 1 cảm biến hoạt động
  - LED nhấp nháy: không có cảm biến nào

### 4. SystemMonitorTask (Priority: Low)
- **Stack size**: 512 words  
- **Chu kỳ**: 60 giây
- **Chức năng**:
  - Gửi heartbeat message
  - Cập nhật system status
  - Giám sát stack usage của các tasks

## Định dạng dữ liệu UART

### JSON Format (Mặc định)
```json
{
  "device":"STM32F411_RTOS",
  "timestamp":123456,
  "temp":25.6,
  "humi":65.2,
  "pres":1013.2,
  "co2":400,
  "tvoc":25,
  "bme":"OK",
  "ccs":"OK"
}
```

### CSV Format
```
123456,25.6,65.2,1013.2,400,25
```

### Plain Text Format
```
TEMP:25.6 HUMI:65.2 PRES:1013.2 CO2:400 TVOC:25
```

## System Messages

### Khởi động
```json
{"type":"SYSTEM_START","device":"STM32F411_RTOS"}
```

### Trạng thái khởi tạo cảm biến
```json
{"type":"INIT","bme280":true,"ccs811":true}
```

### Heartbeat (60s)
```json
{
  "type":"HEARTBEAT",
  "uptime":3600,
  "bme280":true,
  "ccs811":true,
  "errors":0
}
```

## Tối ưu hóa cho STM32F411CE

### Memory Management
- **Total heap**: 15KB
- **Stack sizes** được tối ưu cho từng task
- Sử dụng static buffers thay vì dynamic allocation
- Queue sizes nhỏ để tiết kiệm RAM

### Performance
- BME280: x1 oversampling để giảm thời gian đọc
- CCS811: 1 second measurement mode
- I2C timeout: 1 second
- UART timeout: 100ms

### Error Handling
- Automatic sensor re-initialization on failure
- Mutex timeout protection
- Error counting và reporting
- Hardware watchdog ready

## Biên dịch và Flash

1. Mở project trong STM32CubeIDE
2. Build project (Release mode khuyến khích)
3. Flash lên STM32F411CEU6
4. Kết nối UART2 (PA2/PA3) với Orange Pi 4A
5. Kết nối I2C2 (PB3/PB10) với BME280 và CCS811

## Giao tiếp với Orange Pi 4A

### Baudrate: 115200
### Commands (có thể mở rộng):
- `GET_STATUS`: Lấy trạng thái hệ thống
- `SET_FORMAT_JSON`: Đặt format JSON
- `SET_FORMAT_CSV`: Đặt format CSV
- `SET_FORMAT_PLAIN`: Đặt format Plain text
- `RESTART`: Khởi động lại hệ thống

## Files chính
- `main.c`: FreeRTOS tasks và main logic
- `BME280_STM32.h/.c`: BME280 driver
- `CCS811.h`, `CCS811_Basic.c`: CCS811 driver
- `FreeRTOSConfig.h`: FreeRTOS configuration

## Lưu ý kỹ thuật
- Sử dụng HAL I2C với 7-bit addressing (shifted)
- CCS811 environmental compensation từ BME280
- LED PC13 active LOW (common trên STM32F411 Black Pill)
- Thread-safe với mutexes cho I2C và UART
- Compatible với STM32F411CE (512KB Flash, 128KB RAM)