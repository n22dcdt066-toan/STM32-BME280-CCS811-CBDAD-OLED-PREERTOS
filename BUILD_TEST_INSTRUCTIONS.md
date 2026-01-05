# Build Test Instructions

## Test build sau khi sửa lỗi linking

### Lỗi đã sửa:
✅ **CCS811 I2C2 reference**: Đã thêm lại `hi2c2` handle và `MX_I2C2_Init()` function

### Files đã cập nhật:
- ✅ `main.c`: Thêm I2C2 handle, function prototype, và initialization
- ✅ Project giờ support cả SPI3 (BME280) và I2C2 (CCS811)

### Peripherals Configuration:
- **I2C2**: Cho CCS811 sensor (100kHz standard mode)
- **SPI3**: Cho BME280 sensor với pin assignments:
  - PB12 → SPI3_SCK
  - PB5 → SPI3_MOSI 
  - PB4 → SPI3_MISO
  - PB13 → CS (GPIO Output)

### Build Process:
1. Trong STM32CubeIDE: `Project → Build Project`
2. Hoặc terminal: `make all` (nếu có Makefile)

### Expected Result:
✅ Build successful - No undefined reference errors
✅ Both BME280 (SPI3) and CCS811 (I2C2) drivers linked properly
✅ Ready for hardware testing

### Hardware Setup Required:
- BME280: Connect to SPI3 pins (PB12, PB5, PB4, PB13)  
- CCS811: Connect to I2C2 pins (default I2C2 pins)
- Both sensors: VCC → 3.3V, GND → GND