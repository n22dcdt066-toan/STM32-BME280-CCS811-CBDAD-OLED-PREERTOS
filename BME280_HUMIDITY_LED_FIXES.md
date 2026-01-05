# BME280 Humidity Fix và LED Error Indicator

## Vấn đề đã phát hiện (từ log data):
1. ✅ **BME280 humidity luôn = 0.0**: Cảm biến không đọc được độ ẩm
2. ✅ **Cần LED C13 báo lỗi**: LED nhấp nháy khi sensor lỗi

## Root Cause Analysis:
### BME280 Humidity Issue:
- **Vấn đề**: BME280_Init() và BME280_ReadAll() vẫn sử dụng I2C functions thay vì SPI3
- **Nguyên nhân**: Khi convert từ I2C sang SPI3, một số functions chưa được update đầy đủ

### LED Error Indicator:
- **Yêu cầu**: LED C13 nhấp nháy khi 1 trong 2 sensor lỗi, sáng bình thường khi cả 2 sensor OK

## Fixes Applied:

### 1. BME280.c Fixes:
```c
// ❌ Trước đây (I2C):
HAL_StatusTypeDef BME280_Init(void)
{
    status = BME280_ReadRegister(BME280_REG_ID, &chip_id); // I2C
    // ...
}

// ✅ Sau khi sửa (SPI3):
HAL_StatusTypeDef BME280_Init(void)
{
    status = BME280_SPI_ReadRegister(BME280_REG_CHIP_ID, &chip_id, 1); // SPI3
    // ...
}
```

**Specific Changes:**
- ✅ `BME280_Init()`: Thay tất cả I2C calls → SPI3 calls
- ✅ `BME280_IsConnected()`: Sử dụng SPI3 thay vì I2C
- ✅ `BME280_ReadAll()`: Thay `BME280_ReadRegisters()` → `BME280_SPI_ReadRegister()`
- ✅ Remove old I2C variables và functions
- ✅ Add `calibration_loaded` validation

### 2. LED Error Indicator (main.c):
```c
void SystemMonitorTask(void *argument)
{
  static uint8_t led_blink_counter = 0;
  
  for(;;)
  {
    // Check sensor status
    bool sensor_error = (!systemStatus.bme280_connected || !systemStatus.ccs811_connected);
    
    if (sensor_error) {
      // Blink LED every 5 seconds when sensor error
      led_blink_counter++;
      if (led_blink_counter >= 10) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        led_blink_counter = 0;
      }
    } else {
      // LED ON when all sensors OK  
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
      led_blink_counter = 0;
    }
    // ...
  }
}
```

## Expected Results:

### BME280 Humidity Fix:
- ✅ **Before**: `"humi":0.0` (luôn = 0)
- ✅ **After**: `"humi":45.2` (giá trị thực tế)

### LED Error Behavior:
- 🔴 **Sensor Error**: LED C13 nhấp nháy mỗi 5 giây
- 🟢 **All Sensors OK**: LED C13 sáng liên tục
- ⚡ **Real-time**: Thay đổi ngay lập tức khi sensor status thay đổi

## Hardware Status Expected:
```json
// Khi fix xong:
{"device":"STM32F411_RTOS","timestamp":180000,"temp":30.5,"humi":65.3,"pres":1013.2,"co2":410,"tvoc":2,"bme":"OK","ccs":"OK"}

// LED behavior:
// - BME280 OK + CCS811 OK = LED sáng liên tục  
// - BME280 ERROR hoặc CCS811 ERROR = LED nhấp nháy
```

## Files Modified:
1. ✅ `BME280.c`: Complete SPI3 conversion for all functions
2. ✅ `main.c`: LED error indicator logic in SystemMonitorTask

## Test Instructions:
1. Flash firmware vào STM32F411CEU6
2. Monitor UART output cho humidity values 
3. Test LED behavior:
   - Disconnect BME280 → LED nhấp nháy
   - Reconnect BME280 → LED sáng bình thường
   - Disconnect CCS811 → LED nhấp nháy  
   - Reconnect cả 2 → LED sáng bình thường