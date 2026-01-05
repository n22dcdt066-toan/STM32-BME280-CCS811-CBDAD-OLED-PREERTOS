/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280_STM32.h"
#include "CCS811.h"
#include "BH1750_STM32.h"
#include "ssd1306.h"
#include "fonts.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Sensor data structure for queue
typedef struct {
    float temperature;
    float humidity; 
    float pressure;
    uint16_t co2;
    uint16_t tvoc;
    float light_lux;        // Light intensity from BH1750
    float soil_moisture;    // Soil moisture percentage (0-100%)
    uint16_t soil_adc_raw;  // Raw ADC value from soil sensor
    uint32_t timestamp;
    uint8_t bme_valid : 1;
    uint8_t ccs_valid : 1;
    uint8_t bh1750_valid : 1;  // BH1750 validity flag
    uint8_t soil_valid : 1;    // Soil moisture sensor validity flag
} SensorData_t;

// UART message structure
typedef struct {
    char data[256];
    uint16_t length;
} UartMessage_t;

// System status structure
typedef struct {
    bool bme280_connected;
    bool ccs811_connected;
    bool bh1750_connected;    // BH1750 connection status
    bool soil_sensor_active;  // Soil moisture sensor status
    bool oled_connected;      // OLED display connection status
    uint32_t sensor_errors;
    uint32_t uart_errors;
    uint32_t uptime_seconds;
} SystemStatus_t;

// Data format enumeration
typedef enum {
    FORMAT_JSON,
    FORMAT_CSV,
    FORMAT_PLAIN
} DataFormat_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

// Task stack sizes (words)
#define SENSOR_TASK_STACK_SIZE    512
#define UART_TASK_STACK_SIZE      768
#define LED_TASK_STACK_SIZE       256
#define MONITOR_TASK_STACK_SIZE   512
#define OLED_TASK_STACK_SIZE      512

// Task priorities
#define SENSOR_TASK_PRIORITY      (osPriority_t) osPriorityHigh
#define UART_TASK_PRIORITY        (osPriority_t) osPriorityAboveNormal
#define LED_TASK_PRIORITY         (osPriority_t) osPriorityBelowNormal
#define MONITOR_TASK_PRIORITY     (osPriority_t) osPriorityLow
#define OLED_TASK_PRIORITY        (osPriority_t) osPriorityNormal

// Queue sizes
#define SENSOR_QUEUE_SIZE         5
#define UART_QUEUE_SIZE           10

// Timing constants
#define SENSOR_READ_PERIOD_MS     3000
#define LED_BLINK_PERIOD_MS       500
#define OLED_UPDATE_PERIOD_MS     2000
#define PAGE_SWITCH_PERIOD_MS     3000  // Switch between sensor data pages
#define MONITOR_PERIOD_MS         60000
#define UART_TIMEOUT_MS           100

// Soil Moisture Sensor Configuration
#define SOIL_MOISTURE_ADC_CHANNEL ADC_CHANNEL_4  // PA4 = ADC1_IN4
#define SOIL_MOISTURE_THRESHOLD   30.0           // Minimum moisture percentage for irrigation
#define SOIL_ADC_MAX_VALUE        4095           // 12-bit ADC maximum value
#define SOIL_ADC_MIN_VALUE        0              // ADC minimum value
#define SOIL_MOISTURE_SAMPLES     10             // Number of samples for averaging
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// Global system variables
SystemStatus_t systemStatus = {false, false, false, false, false, 0, 0, 0};

// Global variable to store last sensor reading for button page switching
SensorData_t lastSensorData = {0};
DataFormat_t currentDataFormat = FORMAT_JSON;

// OLED display page switching variables
static uint8_t current_display_page = 0;     // 0=BME280, 1=CCS811+BH1750, 2=Soil+System
static uint32_t last_page_switch_time = 0;   // Last time page was switched
static uint8_t oled_current_page = 0;        // Current OLED page (0-3)
static const uint8_t oled_total_pages = 4;   // Total number of OLED pages

// FreeRTOS task handles
osThreadId_t sensorTaskHandle;
osThreadId_t uartTaskHandle;
osThreadId_t ledTaskHandle;
osThreadId_t monitorTaskHandle;
osThreadId_t oledTaskHandle;

// FreeRTOS Objects
osMessageQueueId_t sensorDataQueueHandle;
osMessageQueueId_t uartTxQueueHandle;
osMutexId_t spiMutexHandle;
osMutexId_t uartMutexHandle;
osMutexId_t oledMutexHandle;

// Sensor data buffers
BME280_Data_t bme280_data;
CCS811_Data_t ccs811_data;
BH1750_Data_t bh1750_data;

// UART transmission buffer
char uart_tx_buffer[512];

// FreeRTOS task attributes
const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = SENSOR_TASK_STACK_SIZE * 4,
  .priority = SENSOR_TASK_PRIORITY,
};

const osThreadAttr_t uartTask_attributes = {
  .name = "UartTask", 
  .stack_size = UART_TASK_STACK_SIZE * 4,
  .priority = UART_TASK_PRIORITY,
};

const osThreadAttr_t ledTask_attributes = {
  .name = "LedTask",
  .stack_size = LED_TASK_STACK_SIZE * 4,
  .priority = LED_TASK_PRIORITY,
};

const osThreadAttr_t monitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = MONITOR_TASK_STACK_SIZE * 4,
  .priority = MONITOR_TASK_PRIORITY,
};

const osThreadAttr_t oledTask_attributes = {
  .name = "OledTask",
  .stack_size = OLED_TASK_STACK_SIZE * 4,
  .priority = OLED_TASK_PRIORITY,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// Task function prototypes
void SensorReadingTask(void *argument);
void UartCommunicationTask(void *argument); 
void LedStatusTask(void *argument);
void SystemMonitorTask(void *argument);
void OledDisplayTask(void *argument);

// Helper function prototypes
void InitializeSensors(void);
void ReadAllSensorData(SensorData_t* data);
void FormatSensorData(SensorData_t* data, char* buffer, size_t buffer_size);
void ProcessUartCommand(const char* command);
HAL_StatusTypeDef SendUartMessage(const char* message);
void UpdateSystemStatus(void);
void I2C_Scanner(void);
void DisplaySensorDataOnOled(SensorData_t* data);

// LED status functions
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_SetSensorStatus(bool sensors_ok);

// Button handling functions
void Button_Init(void);
bool Button_IsPressed(void);
void Button_HandlePageSwitch(void);

// Soil moisture sensor functions
uint32_t ReadSoilMoistureSensor(void);
float ConvertSoilMoistureToPercentage(uint32_t adc_value);
void InitializeSoilSensor(void);
bool IsSoilMoistureLow(float soil_moisture_percent);
void CheckIrrigationNeeds(SensorData_t* data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  // Initialize LED pin
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // LED OFF initially (active low)
  
  // Initialize button (already configured in MX_GPIO_Init)
  Button_Init();
  
  // Initialize OLED page switching
  current_display_page = 0;
  last_page_switch_time = HAL_GetTick();
  
  // Print system information
  char startup_msg[] = "=== STM32F411 + BME280 + CCS811 + BH1750 + Soil Sensor + FreeRTOS ===\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startup_msg, strlen(startup_msg), 1000);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  // Create SPI mutex for sensor access synchronization
  spiMutexHandle = osMutexNew(NULL);
  if (spiMutexHandle == NULL) {
    Error_Handler();
  }
  
  // Create UART mutex for UART access synchronization
  uartMutexHandle = osMutexNew(NULL);
  if (uartMutexHandle == NULL) {
    Error_Handler();
  }
  
  // Create OLED mutex for I2C OLED access synchronization
  oledMutexHandle = osMutexNew(NULL);
  if (oledMutexHandle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // Create sensor data queue
  sensorDataQueueHandle = osMessageQueueNew(SENSOR_QUEUE_SIZE, sizeof(SensorData_t), NULL);
  if (sensorDataQueueHandle == NULL) {
    Error_Handler();
  }
  
  // Create UART transmission queue
  uartTxQueueHandle = osMessageQueueNew(UART_QUEUE_SIZE, sizeof(UartMessage_t), NULL);
  if (uartTxQueueHandle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Create sensor reading task
  sensorTaskHandle = osThreadNew(SensorReadingTask, NULL, &sensorTask_attributes);
  if (sensorTaskHandle == NULL) {
    Error_Handler();
  }
  
  // Create UART communication task
  uartTaskHandle = osThreadNew(UartCommunicationTask, NULL, &uartTask_attributes);
  if (uartTaskHandle == NULL) {
    Error_Handler();
  }
  
  // Create LED status task
  ledTaskHandle = osThreadNew(LedStatusTask, NULL, &ledTask_attributes);
  if (ledTaskHandle == NULL) {
    Error_Handler();
  }
  
  // Create system monitor task
  monitorTaskHandle = osThreadNew(SystemMonitorTask, NULL, &monitorTask_attributes);
  if (monitorTaskHandle == NULL) {
    Error_Handler();
  }
  
  // Create OLED display task
  oledTaskHandle = osThreadNew(OledDisplayTask, NULL, &oledTask_attributes);
  if (oledTaskHandle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up for active low button
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  // Configure PB13 as BME280 CS pin (SPI3 Chip Select)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Set CS high initially
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Initialize BME280, CCS811, BH1750, and Soil Moisture sensors
  * @param  None
  * @retval None
  */
void InitializeSensors(void)
{
  systemStatus.bme280_connected = false;
  systemStatus.ccs811_connected = false;
  systemStatus.bh1750_connected = false;
  systemStatus.soil_sensor_active = false;
  systemStatus.oled_connected = false;
  
  // Initialize BME280 with SPI3
  if (BME280_Init() == HAL_OK) {
    systemStatus.bme280_connected = true;
  }
  
  // Initialize CCS811 with I2C2
  if (CCS811_Init() == HAL_OK) {
    systemStatus.ccs811_connected = true;
    // Set drive mode to 1 second intervals
    CCS811_SetDriveMode(CCS811_DRIVE_MODE_1SEC);
  }
  
  // Initialize BH1750 with I2C2 (default address 0x23)
  if (BH1750_Init(BH1750_DEFAULT_ADDRESS) == BH1750_OK) {
    systemStatus.bh1750_connected = true;
  }
  
  // Initialize Soil Moisture Sensor (ADC1)
  InitializeSoilSensor();
  
  // Initialize OLED with I2C3
  if (SSD1306_Init() == 1) {
    systemStatus.oled_connected = true;
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_UpdateScreen();
    SSD1306_GotoXY(2, 2);
    SSD1306_Puts("STM32 Sensor", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(2, 25);
    SSD1306_Puts("Monitor", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
    HAL_Delay(2000);
  }
}

/**
  * @brief  Read all sensor data
  * @param  data: Pointer to SensorData_t structure
  * @retval None
  */
void ReadAllSensorData(SensorData_t* data)
{
  if (data == NULL) return;
  
  // Initialize data structure
  data->bme_valid = 0;
  data->ccs_valid = 0;  
  data->bh1750_valid = 0;
  data->soil_valid = 0;
  data->timestamp = HAL_GetTick();
  
  // Read BME280 data (SPI3)
  if (systemStatus.bme280_connected) {
    if (BME280_ReadAll(&bme280_data) == HAL_OK) {
      data->temperature = bme280_data.temperature;
      data->humidity = bme280_data.humidity;
      data->pressure = bme280_data.pressure;
      data->bme_valid = 1;
    }
  }
  
  // Read CCS811 data (I2C2) 
  if (systemStatus.ccs811_connected) {
    if (CCS811_ReadData(&ccs811_data) == HAL_OK) {
      data->co2 = ccs811_data.co2;
      data->tvoc = ccs811_data.tvoc;
      data->ccs_valid = 1;
      
      // Set environmental compensation for CCS811 from BME280 data
      if (data->bme_valid) {
        CCS811_SetEnvironmentalData(data->humidity, data->temperature);
      }
    }
  }
  
  // Read BH1750 light sensor data (I2C2)
  if (systemStatus.bh1750_connected) {
    if (BH1750_ReadAll(&bh1750_data) == BH1750_OK) {
      data->light_lux = bh1750_data.lux;
      data->bh1750_valid = 1;
    }
  }
  
  // Read Soil Moisture Sensor data (ADC1)
  if (systemStatus.soil_sensor_active) {
    data->soil_adc_raw = ReadSoilMoistureSensor();
    if (data->soil_adc_raw > 0) {
      data->soil_moisture = ConvertSoilMoistureToPercentage(data->soil_adc_raw);
      data->soil_valid = 1;
    }
  }
}

/**
  * @brief  Format sensor data for transmission
  * @param  data: Pointer to sensor data
  * @param  buffer: Buffer to store formatted string
  * @param  buffer_size: Size of buffer
  * @retval None
  */
void FormatSensorData(SensorData_t* data, char* buffer, size_t buffer_size)
{
  if (data == NULL || buffer == NULL) return;
  
  switch (currentDataFormat) {
    case FORMAT_JSON:
      snprintf(buffer, buffer_size,
               "{\"device\":\"STM32F411_RTOS\",\"timestamp\":%lu,\"temp\":%.1f,\"humi\":%.1f,\"pres\":%.1f,\"co2\":%d,\"tvoc\":%d,\"lux\":%.1f,\"soil\":%.1f,\"soil_adc\":%d,\"bme\":\"%s\",\"ccs\":\"%s\",\"bh1750\":\"%s\",\"soil_sensor\":\"%s\"}\r\n",
               data->timestamp,
               data->temperature, data->humidity, data->pressure,
               data->co2, data->tvoc, data->light_lux, data->soil_moisture, data->soil_adc_raw,
               data->bme_valid ? "OK" : "ERR",
               data->ccs_valid ? "OK" : "ERR", 
               data->bh1750_valid ? "OK" : "ERR",
               data->soil_valid ? "OK" : "ERR");
      break;
      
    case FORMAT_CSV:
      snprintf(buffer, buffer_size,
               "%lu,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d\r\n",
               data->timestamp,
               data->temperature, data->humidity, data->pressure,
               data->co2, data->tvoc, data->light_lux, data->soil_moisture, data->soil_adc_raw,
               data->bme_valid, data->ccs_valid, data->bh1750_valid, data->soil_valid);
      break;
      
    case FORMAT_PLAIN:
    default:
      snprintf(buffer, buffer_size,
               "Temp: %.1f°C, Humidity: %.1f%%, Pressure: %.1f hPa, CO2: %d ppm, TVOC: %d ppb, Light: %.1f lux, Soil: %.1f%% (%s)\r\n",
               data->temperature, data->humidity, data->pressure,
               data->co2, data->tvoc, data->light_lux, data->soil_moisture,
               (data->soil_moisture < SOIL_MOISTURE_THRESHOLD) ? "DRY" : "WET");
      break;
  }
}

/**
  * @brief  Process UART command
  * @param  command: Command string
  * @retval None
  */
void ProcessUartCommand(const char* command)
{
  if (command == NULL) return;
  
  if (strncmp(command, "GET_STATUS", 10) == 0) {
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "{\"type\":\"STATUS\",\"bme280\":%s,\"ccs811\":%s,\"bh1750\":%s,\"soil_sensor\":%s,\"uptime\":%lu,\"errors\":%lu}\r\n",
             systemStatus.bme280_connected ? "true" : "false",
             systemStatus.ccs811_connected ? "true" : "false",
             systemStatus.bh1750_connected ? "true" : "false",
             systemStatus.soil_sensor_active ? "true" : "false",
             systemStatus.uptime_seconds,
             systemStatus.sensor_errors);
    SendUartMessage(uart_tx_buffer);
  }
  else if (strncmp(command, "SET_FORMAT_JSON", 15) == 0) {
    currentDataFormat = FORMAT_JSON;
    SendUartMessage("OK: Format set to JSON\r\n");
  }
  else if (strncmp(command, "SET_FORMAT_CSV", 14) == 0) {
    currentDataFormat = FORMAT_CSV;
    SendUartMessage("OK: Format set to CSV\r\n");
  }
  else if (strncmp(command, "SET_FORMAT_PLAIN", 16) == 0) {
    currentDataFormat = FORMAT_PLAIN;
    SendUartMessage("OK: Format set to PLAIN\r\n");
  }
  else if (strncmp(command, "GET_SOIL_STATUS", 15) == 0) {
    // Read current soil moisture
    uint32_t soil_adc = ReadSoilMoistureSensor();
    float soil_percent = ConvertSoilMoistureToPercentage(soil_adc);
    
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "{\"type\":\"SOIL_STATUS\",\"moisture\":%.1f,\"adc\":%lu,\"threshold\":%.1f,\"needs_water\":%s}\r\n",
             soil_percent, soil_adc, SOIL_MOISTURE_THRESHOLD,
             (soil_percent < SOIL_MOISTURE_THRESHOLD) ? "true" : "false");
    SendUartMessage(uart_tx_buffer);
  }
  else if (strncmp(command, "CALIBRATE_SOIL", 14) == 0) {
    // Calibration helper - read current ADC value for calibration
    uint32_t soil_adc = ReadSoilMoistureSensor();
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "Current soil ADC value: %lu (use this for calibration)\r\n", soil_adc);
    SendUartMessage(uart_tx_buffer);
  }
  else if (strncmp(command, "RESTART", 7) == 0) {
    SendUartMessage("OK: Restarting system\r\n");
    HAL_Delay(100);
    NVIC_SystemReset();
  }
  else {
    SendUartMessage("ERR: Unknown command\r\n");
  }
}

/**
  * @brief  Send UART message
  * @param  message: Message to send
  * @retval HAL status
  */
HAL_StatusTypeDef SendUartMessage(const char* message)
{
  if (message == NULL) return HAL_ERROR;
  
  return HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), UART_TIMEOUT_MS);
}

/**
  * @brief  Initialize soil moisture sensor (ADC1)
  * @param  None
  * @retval None
  */
void InitializeSoilSensor(void)
{
  // ADC1 is already initialized by HAL_ADC_MspInit
  // Just mark soil sensor as active
  systemStatus.soil_sensor_active = true;
}

/**
  * @brief  Read soil moisture sensor via ADC
  * @param  None
  * @retval ADC raw value (0-4095)
  */
uint32_t ReadSoilMoistureSensor(void)
{
  uint32_t adc_value = 0;
  uint32_t sum = 0;
  
  // STM32F4 series doesn't need manual calibration
  // ADC is automatically calibrated during initialization
  
  // Read multiple samples for averaging
  for (int i = 0; i < SOIL_MOISTURE_SAMPLES; i++) {
    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
      // Wait for conversion to complete
      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        // Get ADC value
        sum += HAL_ADC_GetValue(&hadc1);
      }
      HAL_ADC_Stop(&hadc1);
    }
    HAL_Delay(10); // Small delay between readings
  }
  
  adc_value = sum / SOIL_MOISTURE_SAMPLES;
  return adc_value;
}

/**
  * @brief  Convert ADC value to soil moisture percentage
  * @param  adc_value: Raw ADC value (0-4095)
  * @retval Soil moisture percentage (0-100%)
  */
float ConvertSoilMoistureToPercentage(uint32_t adc_value)
{
  // Convert ADC to percentage
  // Higher ADC value = higher voltage = drier soil
  // Lower ADC value = lower voltage = wetter soil
  float percentage = 100.0f - ((float)adc_value / SOIL_ADC_MAX_VALUE) * 100.0f;
  
  // Ensure percentage is within 0-100% range
  if (percentage < 0.0f) percentage = 0.0f;
  if (percentage > 100.0f) percentage = 100.0f;
  
  return percentage;
}

/**
  * @brief  Check if soil moisture is low and needs irrigation
  * @param  soil_moisture_percent: Soil moisture percentage
  * @retval true if irrigation needed, false otherwise
  */
bool IsSoilMoistureLow(float soil_moisture_percent)
{
  return (soil_moisture_percent < SOIL_MOISTURE_THRESHOLD);
}

/**
  * @brief  Check irrigation needs and send alerts
  * @param  data: Sensor data structure
  * @retval None
  */
void CheckIrrigationNeeds(SensorData_t* data)
{
  if (data == NULL || !data->soil_valid) return;
  
  static bool last_irrigation_needed = false;
  bool irrigation_needed = IsSoilMoistureLow(data->soil_moisture);
  
  // Send alert only when status changes
  if (irrigation_needed != last_irrigation_needed) {
    UartMessage_t alertMsg;
    
    if (irrigation_needed) {
      snprintf(alertMsg.data, sizeof(alertMsg.data),
               "{\"type\":\"IRRIGATION_ALERT\",\"soil_moisture\":%.1f,\"threshold\":%.1f,\"status\":\"NEEDED\"}\r\n",
               data->soil_moisture, SOIL_MOISTURE_THRESHOLD);
    } else {
      snprintf(alertMsg.data, sizeof(alertMsg.data),
               "{\"type\":\"IRRIGATION_ALERT\",\"soil_moisture\":%.1f,\"threshold\":%.1f,\"status\":\"OK\"}\r\n",
               data->soil_moisture, SOIL_MOISTURE_THRESHOLD);
    }
    
    alertMsg.length = strlen(alertMsg.data);
    osMessageQueuePut(uartTxQueueHandle, &alertMsg, 0, 50);
    
    last_irrigation_needed = irrigation_needed;
  }
}

/**
  * @brief  Display sensor data on OLED screen with page switching
  * @param  data: Pointer to SensorData_t structure
  * @retval None
  */
void DisplaySensorDataOnOled(SensorData_t* data)
{
  if (!systemStatus.oled_connected) {
    return;
  }
  
  static uint32_t last_page_switch = 0;
  char buffer[32];
  
  // Switch page every 4 seconds (longer time to read data)
  if ((HAL_GetTick() - last_page_switch) > 4000) {
    oled_current_page = (oled_current_page + 1) % oled_total_pages;
    last_page_switch = HAL_GetTick();
  }
  
  // Clear screen
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  
  switch(oled_current_page) {
    case 0: // BME280 Data Page
      SSD1306_GotoXY(10, 0);
      SSD1306_Puts("-- BME280 --", &Font_7x10, SSD1306_COLOR_WHITE);
      
      if (systemStatus.bme280_connected && data->bme_valid) {
        SSD1306_GotoXY(0, 15);
        snprintf(buffer, sizeof(buffer), "Temp: %.1f C", data->temperature);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        
        SSD1306_GotoXY(0, 27);
        snprintf(buffer, sizeof(buffer), "Humi: %.1f %%", data->humidity);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        
        SSD1306_GotoXY(0, 39);
        snprintf(buffer, sizeof(buffer), "Pres: %.0f hPa", data->pressure);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
      } else {
        SSD1306_GotoXY(15, 25);
        if (!systemStatus.bme280_connected) {
          SSD1306_Puts("DISCONNECTED", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
          SSD1306_Puts("NO DATA", &Font_7x10, SSD1306_COLOR_WHITE);
        }
      }
      break;
      
    case 1: // CCS811 Data Page
      SSD1306_GotoXY(10, 0);
      SSD1306_Puts("-- CCS811 --", &Font_7x10, SSD1306_COLOR_WHITE);
      
      if (systemStatus.ccs811_connected && data->ccs_valid) {
        SSD1306_GotoXY(0, 15);
        snprintf(buffer, sizeof(buffer), "CO2:  %d ppm", data->co2);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        
        SSD1306_GotoXY(0, 27);
        snprintf(buffer, sizeof(buffer), "TVOC: %d ppb", data->tvoc);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        
        // Show data freshness
        SSD1306_GotoXY(0, 39);
        SSD1306_Puts("Air Quality", &Font_7x10, SSD1306_COLOR_WHITE);
      } else {
        SSD1306_GotoXY(15, 25);
        if (!systemStatus.ccs811_connected) {
          SSD1306_Puts("DISCONNECTED", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
          SSD1306_Puts("NO DATA", &Font_7x10, SSD1306_COLOR_WHITE);
        }
      }
      break;
      
    case 2: // BH1750 Data Page
      SSD1306_GotoXY(10, 0);
      SSD1306_Puts("-- BH1750 --", &Font_7x10, SSD1306_COLOR_WHITE);
      
      if (systemStatus.bh1750_connected && data->bh1750_valid) {
        SSD1306_GotoXY(0, 20);
        snprintf(buffer, sizeof(buffer), "Light: %.1f lux", data->light_lux);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        
        SSD1306_GotoXY(0, 35);
        if (data->light_lux < 10) {
          SSD1306_Puts("Dark", &Font_7x10, SSD1306_COLOR_WHITE);
        } else if (data->light_lux < 100) {
          SSD1306_Puts("Dim", &Font_7x10, SSD1306_COLOR_WHITE);
        } else if (data->light_lux < 1000) {
          SSD1306_Puts("Normal", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
          SSD1306_Puts("Bright", &Font_7x10, SSD1306_COLOR_WHITE);
        }
      } else {
        SSD1306_GotoXY(15, 25);
        if (!systemStatus.bh1750_connected) {
          SSD1306_Puts("DISCONNECTED", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
          SSD1306_Puts("NO DATA", &Font_7x10, SSD1306_COLOR_WHITE);
        }
      }
      break;
      
    case 3: // Soil Sensor Data Page
      SSD1306_GotoXY(8, 0);
      SSD1306_Puts("-- SOIL --", &Font_7x10, SSD1306_COLOR_WHITE);
      
      if (systemStatus.soil_sensor_active && data->soil_valid) {
        SSD1306_GotoXY(0, 15);
        snprintf(buffer, sizeof(buffer), "Moist: %.1f%%", data->soil_moisture);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
        
        SSD1306_GotoXY(0, 27);
        if (data->soil_moisture < SOIL_MOISTURE_THRESHOLD) {
          SSD1306_Puts("Status: DRY", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
          SSD1306_Puts("Status: WET", &Font_7x10, SSD1306_COLOR_WHITE);
        }
        
        SSD1306_GotoXY(0, 39);
        snprintf(buffer, sizeof(buffer), "Raw: %d", data->soil_adc_raw);
        SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
      } else {
        SSD1306_GotoXY(20, 25);
        if (!systemStatus.soil_sensor_active) {
          SSD1306_Puts("INACTIVE", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
          SSD1306_Puts("NO DATA", &Font_7x10, SSD1306_COLOR_WHITE);
        }
      }
      break;
  }
  
  // Status bar at bottom
  SSD1306_GotoXY(0, 55);
  snprintf(buffer, sizeof(buffer), "P%d/%d", oled_current_page + 1, oled_total_pages);
  SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
  
  // Show uptime in corner
  SSD1306_GotoXY(85, 55);
  uint32_t uptime_sec = HAL_GetTick() / 1000;
  if (uptime_sec < 60) {
    snprintf(buffer, sizeof(buffer), "%lus", uptime_sec);
  } else if (uptime_sec < 3600) {
    snprintf(buffer, sizeof(buffer), "%lum", uptime_sec / 60);
  } else {
    snprintf(buffer, sizeof(buffer), "%luh", uptime_sec / 3600);
  }
  SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
  
  // Update screen
  SSD1306_UpdateScreen();
}

/**
  * @brief  Update system status
  * @retval None
  */
void UpdateSystemStatus(void)
{
  systemStatus.uptime_seconds = HAL_GetTick() / 1000;
  
  // Update connection status by checking sensors
  systemStatus.bme280_connected = BME280_IsConnected();
  systemStatus.ccs811_connected = CCS811_IsConnected();  
  systemStatus.bh1750_connected = BH1750_IsConnected();
  
  // Check OLED connection via I2C3
  systemStatus.oled_connected = (HAL_I2C_IsDeviceReady(&hi2c3, SSD1306_I2C_ADDR, 1, 50) == HAL_OK);
  
  // Soil sensor is always active if ADC is working (no specific connection check needed)
}

/**
  * @brief  Turn on status LED (PC13)
  * @retval None
  */
void LED_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // LED is active low
}

/**
  * @brief  Turn off status LED (PC13)
  * @retval None
  */
void LED_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // LED is active low
}

/**
  * @brief  Toggle status LED (PC13)
  * @retval None
  */
void LED_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/**
  * @brief  Set LED status based on sensor health
  * @param  sensors_ok: true if all sensors working, false if any sensor error
  * @retval None
  */
void LED_SetSensorStatus(bool sensors_ok)
{
  static uint32_t last_blink_time = 0;
  static bool blink_state = false;
  
  if (sensors_ok) {
    // All sensors OK - LED solid on
    LED_On();
  } else {
    // Sensor error - LED blink every 500ms
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_blink_time >= 500) {
      if (blink_state) {
        LED_Off();
      } else {
        LED_On();
      }
      blink_state = !blink_state;
      last_blink_time = current_time;
    }
  }
}

/**
  * @brief  Initialize button (already done in MX_GPIO_Init)
  * @retval None
  */
void Button_Init(void)
{
  // Button initialization is already done in MX_GPIO_Init()
  // PA0 configured as input with pull-up resistor
}

/**
  * @brief  Check if button is pressed (with debounce)
  * @retval true if button pressed, false otherwise
  */
bool Button_IsPressed(void)
{
  static uint32_t last_press_time = 0;
  static bool last_stable_state = true; // Button not pressed initially (active low)
  static bool button_pressed_flag = false;
  
  bool current_state = BUTTON_PRESSED();
  uint32_t current_time = HAL_GetTick();
  
  // Debounce logic - 50ms debounce time
  if (current_state != last_stable_state) {
    if ((current_time - last_press_time) > 50) {
      last_stable_state = current_state;
      last_press_time = current_time;
      
      // Detect falling edge (button press for active low)
      if (current_state == true && !button_pressed_flag) {
        button_pressed_flag = true;
        return true; // Button just pressed
      }
    }
  }
  
  // Reset flag when button is released
  if (current_state == false) {
    button_pressed_flag = false;
  }
  
  return false; // No new press detected
}

/**
  * @brief  Handle page switching when button is pressed
  * @retval None
  */
void Button_HandlePageSwitch(void)
{
  if (Button_IsPressed()) {
    // Move to next page
    current_display_page = (current_display_page + 1) % 4; // 4 pages total
    last_page_switch_time = HAL_GetTick();
    
    // Immediately update OLED with current page
    if (systemStatus.oled_connected) {
      if (osMutexAcquire(oledMutexHandle, 100) == osOK) {
        // Force immediate display update using last sensor data
        DisplaySensorDataOnOled(&lastSensorData);
        osMutexRelease(oledMutexHandle);
      }
    }
    
    // Provide visual feedback via LED blink
    for (int i = 0; i < 2; i++) {
      LED_Toggle();
      osDelay(50);
      LED_Toggle();
      osDelay(50);
    }
  }
}

/**
  * @brief  I2C Scanner - Scans all I2C addresses to find connected devices
  * @param  None
  * @retval None
  */
void I2C_Scanner(void)
{
  char msg[100];
  uint8_t devices_found = 0;
  
  SendUartMessage("Starting I2C Scanner on I2C3...\r\n");
  
  for (uint8_t address = 1; address < 128; address++) {
    // Test if device responds at this address
    if (HAL_I2C_IsDeviceReady(&hi2c3, address << 1, 1, 50) == HAL_OK) {
      snprintf(msg, sizeof(msg), "Device found at address 0x%02X (7-bit: 0x%02X)\r\n", 
               address << 1, address);
      SendUartMessage(msg);
      devices_found++;
    }
  }
  
  if (devices_found == 0) {
    SendUartMessage("No I2C devices found!\r\n");
    SendUartMessage("Check connections and pull-up resistors.\r\n");
  } else {
    snprintf(msg, sizeof(msg), "I2C scan complete. %d device(s) found.\r\n", devices_found);
    SendUartMessage(msg);
  }
}

/**
  * @brief  Sensor Reading Task - Reads BME280, CCS811, and BH1750 sensors
  * @param  argument: Not used
  * @retval None
  */
void SensorReadingTask(void *argument)
{
  SensorData_t sensorData;
  osStatus_t status;
  
  // Initialize sensors
  InitializeSensors();
  
  // Send initialization status
  UartMessage_t initMsg;
  snprintf(initMsg.data, sizeof(initMsg.data),
           "{\"type\":\"INIT\",\"bme280\":%s,\"ccs811\":%s,\"bh1750\":%s,\"soil_sensor\":%s}\r\n",
           systemStatus.bme280_connected ? "true" : "false",
           systemStatus.ccs811_connected ? "true" : "false",
           systemStatus.bh1750_connected ? "true" : "false",
           systemStatus.soil_sensor_active ? "true" : "false");
  initMsg.length = strlen(initMsg.data);
  
  status = osMessageQueuePut(uartTxQueueHandle, &initMsg, 0, 0);
  
  for(;;)
  {
    // Acquire SPI mutex for sensor access
    if (osMutexAcquire(spiMutexHandle, 1000) == osOK) {
      ReadAllSensorData(&sensorData);
      
      // Update global sensor data for button page switching
      lastSensorData = sensorData;
      
      osMutexRelease(spiMutexHandle);
      
      // Update LED status based on sensor health
      bool sensors_ok = (systemStatus.bme280_connected || systemStatus.ccs811_connected || 
                        systemStatus.bh1750_connected || systemStatus.soil_sensor_active);
      
      // Check if any connected sensor has valid data
      bool valid_data = false;
      if (systemStatus.bme280_connected && sensorData.bme_valid) valid_data = true;
      if (systemStatus.ccs811_connected && sensorData.ccs_valid) valid_data = true;
      if (systemStatus.bh1750_connected && sensorData.bh1750_valid) valid_data = true;
      if (systemStatus.soil_sensor_active && sensorData.soil_valid) valid_data = true;
      
      // LED control: solid if sensors OK and have valid data, blink if problems
      LED_SetSensorStatus(sensors_ok && valid_data);
      
      // Update OLED display immediately with fresh data
      if (systemStatus.oled_connected) {
        if (osMutexAcquire(oledMutexHandle, 100) == osOK) {
          DisplaySensorDataOnOled(&sensorData);
          osMutexRelease(oledMutexHandle);
        }
      }
      
      // Check irrigation needs based on sensor data
      CheckIrrigationNeeds(&sensorData);
      
      // Put data in queue for UART transmission
      status = osMessageQueuePut(sensorDataQueueHandle, &sensorData, 0, 100);
      
      if (status != osOK) {
        systemStatus.sensor_errors++;
      }
    } else {
      systemStatus.sensor_errors++;
    }
    
    // Check button for page switching
    Button_HandlePageSwitch();
    
    osDelay(SENSOR_READ_PERIOD_MS);
  }
}

/**
  * @brief  UART Communication Task - Handles UART transmission
  * @param  argument: Not used  
  * @retval None
  */
void UartCommunicationTask(void *argument)
{
  SensorData_t receivedData;
  UartMessage_t receivedMsg;
  osStatus_t status;
  
  for(;;)
  {
    // Check for sensor data to transmit
    status = osMessageQueueGet(sensorDataQueueHandle, &receivedData, NULL, 50);
    if (status == osOK) {
      UartMessage_t txMsg;
      
      // Acquire UART mutex
      if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
        FormatSensorData(&receivedData, txMsg.data, sizeof(txMsg.data));
        txMsg.length = strlen(txMsg.data);
        
        // Send via UART
        HAL_StatusTypeDef uart_status = HAL_UART_Transmit(&huart2, 
                                                          (uint8_t*)txMsg.data, 
                                                          txMsg.length, 
                                                          UART_TIMEOUT_MS);
        osMutexRelease(uartMutexHandle);
        
        if (uart_status != HAL_OK) {
          systemStatus.uart_errors++;
        }
      } else {
        systemStatus.uart_errors++;
      }
    }
    
    // Check for UART messages to send 
    status = osMessageQueueGet(uartTxQueueHandle, &receivedMsg, NULL, 50);
    if (status == osOK) {
      if (osMutexAcquire(uartMutexHandle, 1000) == osOK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)receivedMsg.data, receivedMsg.length, UART_TIMEOUT_MS);
        osMutexRelease(uartMutexHandle);
      }
    }
    
    osDelay(50); // Small delay for responsiveness
  }
}

/**
  * @brief  LED Status Task - Controls LED based on sensor status
  * @param  argument: Not used
  * @retval None
  */
void LedStatusTask(void *argument)
{
  bool ledState = false;
  
  for(;;)
  {
    // Check if any sensor is connected
    bool anySensorConnected = systemStatus.bme280_connected || 
                             systemStatus.ccs811_connected || 
                             systemStatus.bh1750_connected ||
                             systemStatus.soil_sensor_active;
    
    if (anySensorConnected) {
      // At least one sensor working - LED ON
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // Active LOW
    } else {
      // No sensors working - LED BLINK
      ledState = !ledState;
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, ledState ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    
    osDelay(LED_BLINK_PERIOD_MS);
  }
}

/**
  * @brief  System Monitor Task - Updates system status and sends heartbeat
  * @param  argument: Not used
  * @retval None  
  */
void SystemMonitorTask(void *argument)
{
  for(;;)
  {
    UpdateSystemStatus();
    
    // Send heartbeat message
    UartMessage_t heartbeatMsg;
    snprintf(heartbeatMsg.data, sizeof(heartbeatMsg.data),
             "{\"type\":\"HEARTBEAT\",\"uptime\":%lu,\"bme280\":%s,\"ccs811\":%s,\"bh1750\":%s,\"soil_sensor\":%s,\"oled\":%s,\"errors\":%lu}\r\n",
             systemStatus.uptime_seconds,
             systemStatus.bme280_connected ? "true" : "false",
             systemStatus.ccs811_connected ? "true" : "false", 
             systemStatus.bh1750_connected ? "true" : "false",
             systemStatus.soil_sensor_active ? "true" : "false",
             systemStatus.oled_connected ? "true" : "false",
             systemStatus.sensor_errors + systemStatus.uart_errors);
    heartbeatMsg.length = strlen(heartbeatMsg.data);
    
    osMessageQueuePut(uartTxQueueHandle, &heartbeatMsg, 0, 100);
    
    osDelay(MONITOR_PERIOD_MS);
  }
}

/**
  * @brief  OLED Display Task - Displays sensor data on OLED screen
  * @param  argument: Not used
  * @retval None
  */
void OledDisplayTask(void *argument)
{
  // This task is now simplified since OLED display is called directly 
  // from SensorReadingTask after each sensor read
  
  for(;;)
  {
    // Just keep the task alive and update system status periodically
    if (systemStatus.oled_connected) {
      // OLED is working, just delay
      osDelay(5000);
    } else {
      // Try to reinitialize OLED if disconnected
      if (SSD1306_Init() == 1) {
        systemStatus.oled_connected = true;
      }
      osDelay(2000);
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
