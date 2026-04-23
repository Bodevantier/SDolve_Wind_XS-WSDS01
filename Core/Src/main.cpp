/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body - Wind Sensor to N2K Gateway
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
// C modules
extern "C" {
  #include "rs485_wind.h"
  #include "serial_debug.h"
}

// C++ NMEA2000 libraries
#include "NMEA2000.h"
#include "NMEA2000_STM32.hpp"
#include "N2kMessages.h"
#include "N2kTimer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_TOTAL_TO_MS  101  // Total RS485 timeout
#define RS485_PERBYTE_MS   1    // Per-byte timeout
#define WIND_POLL_PERIOD   200  // Poll wind sensor every 200ms (5 Hz)
#define WIND_FILTER_SIZE   3    // Moving average filter size
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Wind sensor data
static float wind_speed_ms = 0.0f;
static float wind_dir_deg = 0.0f;
static uint32_t last_wind_poll = 0;
static uint32_t wind_error_count = 0;
static bool wind_data_valid = false;

// Wind direction filtering (simple moving average)
static float dir_filter[WIND_FILTER_SIZE] = {0};
static uint8_t dir_filter_idx = 0;
static uint8_t dir_filter_valid = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void NMEA2000_Init(tNMEA2000_STM32 *N2k);
void OnN2kOpen();
void GetWindData(float *speed_ms, float *dir_deg, bool *valid);
void NMEA2000_SendWindData(tNMEA2000_STM32 *N2k);
float filter_wind_direction(float new_dir);
void PollWindSensor();
void LED_Update(void);
void LED2_Flash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" CAN_HandleTypeDef hcan;

// ====== LED status ======
// LED1: N2K bus – slow blink while claiming address, solid ON when open
// LED2: Data TX  – brief 50 ms flash on each successful wind PGN send
#define LED2_FLASH_MS      50
static bool     n2k_is_open    = false;
static uint32_t led2_off_tick  = 0;

void LED_Update(void)
{
  // LED1: blink at 1 Hz until N2K is open, then solid ON
  if (n2k_is_open) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  } else {
    GPIO_PinState state = (HAL_GetTick() % 1000u < 500u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
  }

  // LED2: switch off after flash duration expires
  if (led2_off_tick != 0u && HAL_GetTick() >= led2_off_tick) {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    led2_off_tick = 0u;
  }
}

void LED2_Flash(void)
{
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  led2_off_tick = HAL_GetTick() + LED2_FLASH_MS;
}

// ====== Timing configuration ======
#define WIND_UPDATE_PERIOD  100  // ms – PGN 130306 default period per NMEA2000 spec
#define WIND_OFFSET         300  // ms – offset ≥250 ms avoids send failures during address claim window

// ====== Message Scheduler ======
// Disabled at start; OnN2kOpen synchronises it to the library open time.
tN2kSyncScheduler WindScheduler(false, WIND_UPDATE_PERIOD, WIND_OFFSET);

// Rolling SID – ties PGN samples from the same cycle together (0-252, then wraps)
static unsigned char windSID = 0;

/* Simple moving average filter for wind direction */
float filter_wind_direction(float new_dir)
{
  dir_filter[dir_filter_idx] = new_dir;
  dir_filter_idx = (dir_filter_idx + 1) % WIND_FILTER_SIZE;
  if (dir_filter_valid < WIND_FILTER_SIZE) dir_filter_valid++;

  float sum = 0;
  for (uint8_t i = 0; i < dir_filter_valid; i++) {
    sum += dir_filter[i];
  }
  return sum / dir_filter_valid;
}

/* Poll RS485 wind sensor */
void PollWindSensor()
{
  uint32_t now = HAL_GetTick();
  if ((now - last_wind_poll) >= WIND_POLL_PERIOD)
  {
    last_wind_poll = now;

    float raw_speed, raw_dir;
    if (WindRS485_Get(&raw_speed, &raw_dir))
    {
      wind_error_count = 0;  // Reset error counter on success

      // Apply filtering
      wind_speed_ms = raw_speed;  // Speed doesn't need much filtering
      wind_dir_deg = filter_wind_direction(raw_dir);
      wind_data_valid = true;

      // Debug output - convert floats to integers for display
      int speed_int = (int)(wind_speed_ms * 10);  // 2.2 -> 22
      int dir_int = (int)(wind_dir_deg * 10);      // 13.0 -> 130

      static char msg[64];
      snprintf(msg, sizeof(msg), "Wind: %d.%d m/s @ %d.%d deg\r\n",
               speed_int/10, speed_int%10, dir_int/10, dir_int%10);
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    else
    {
      wind_error_count++;
      if (wind_error_count >= 5)  // Report after 5 consecutive failures (1 second)
      {
        static char err_msg[] = "Wind: SENSOR ERROR\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, sizeof(err_msg)-1, 100);
        wind_data_valid = false;
        wind_error_count = 0;  // Reset to avoid spam
      }
    }
  }
}
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize debug serial (USART1)
  SerialDebug_Init(&huart1);
  HAL_UART_Transmit(&huart1, (uint8_t*)"=== Wind Sensor to N2K Gateway ===\r\n", 37, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"Initializing...\r\n", 17, 100);

  // Initialize RS485 wind sensor on USART3 with DE/RE on PB2
  WindRS485_Init(&huart3, USART3_DE_RE_GPIO_Port, USART3_DE_RE_Pin);
  WindRS485_SetTimeouts(RS485_TOTAL_TO_MS, RS485_PERBYTE_MS);
  HAL_UART_Transmit(&huart1, (uint8_t*)"Wind sensor initialized\r\n", 25, 100);

  // Initialize NMEA2000
  tNMEA2000_STM32 N2k(&hcan);
  NMEA2000_Init(&N2k);
  HAL_UART_Transmit(&huart1, (uint8_t*)"N2K network initialized\r\n", 25, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"System ready!\r\n\r\n", 17, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Update LED status indicators
    LED_Update();

    // Poll wind sensor (RS485 – non-blocking, governed by WIND_POLL_PERIOD)
    PollWindSensor();

    // Send wind data to N2K network when scheduler fires
    NMEA2000_SendWindData(&N2k);

    // Process serial debug commands
    SerialDebug_Poll();

    // Service N2K stack: flushes TX buffer, handles ISO requests, heartbeat, etc.
    // Called once per loop iteration at the end – the library docs say
    // "ParseMessages() should be called as often as possible without delays".
    N2k.ParseMessages();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// =====================
// NMEA2000 Init
// =====================
void NMEA2000_Init(tNMEA2000_STM32 *N2k)
{
  // Use STM32 96-bit UID so each physical unit has a unique N2K identity.
  const uint32_t uid0 = *(const volatile uint32_t *)0x1FFFF7E8UL;
  const uint32_t uid1 = *(const volatile uint32_t *)0x1FFFF7ECUL;
  const uint32_t uid2 = *(const volatile uint32_t *)0x1FFFF7F0UL;
  static char serial_code[33];
  snprintf(serial_code, sizeof(serial_code), "%08lX%08lX%08lX",
           (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);
  const uint32_t unique_number = (uid0 ^ uid1 ^ uid2) & 0x1FFFFFUL;

  // Set Product information - Wind sensor gateway
  N2k->SetProductInformation(serial_code,                       // Manufacturer's Model serial code
                             100,                              // Manufacturer's product code
                             "SDolve Wind",                    // Manufacturer's Model ID
                             "1.0.0.1 (2026-01-15)",          // Manufacturer's Software version code
                             "1.0.0.0 (2026-01-15)",          // Manufacturer's Model version
                             1                                 // Load equivalency: 1 × 50 mA = 50 mA
                             );
  
  // Set device information - Atmospheric sensor
  N2k->SetDeviceInformation(unique_number,    // Unique number. Use e.g. Serial number.
                            130,  // Device function=Atmospheric. See codes on NMEA.org
                            85,   // Device class=External Environment
                            2046  // Manufacturer code - free from NMEA.org list
                            );

  // PGN 126998 Configuration Information – responds automatically to ISO requests.
  // Param order: ManufacturerInformation, InstallationDescription1, InstallationDescription2.
  // InstallationDescription fields can be changed at runtime by NMEA 2000 group function.
  N2k->SetConfigurationInformation("SDolve Marine",
                                   "SDolve Wind – masthead anemometer",
                                   "RS-485 to NMEA 2000 bridge");
  
  // Node mode - listen to requests and respond appropriately
  N2k->SetMode(tNMEA2000::N2km_NodeOnly, 23);
  N2k->EnableForward(false);
  
  // Declare which PGNs we transmit
  static const unsigned long TransmitMessages[] PROGMEM = {130306L, 0};  // PGN 130306 = Wind Data
  N2k->ExtendTransmitMessages(TransmitMessages);
  
  // Set callback for when N2K network opens
  N2k->SetOnOpen(OnN2kOpen);
  N2k->Open();
}

// =====================
// OnN2kOpen Callback
// =====================
void OnN2kOpen()
{
  // Start scheduler when N2K bus communication begins
  WindScheduler.UpdateNextTime();
  // LED1 stops blinking and goes solid
  n2k_is_open = true;
}

// =====================
// Get Wind Data
// =====================
void GetWindData(float *speed_ms, float *dir_deg, bool *valid)
{
  *speed_ms = wind_speed_ms;
  *dir_deg = wind_dir_deg;
  *valid = wind_data_valid;
}

// =====================
// Send Wind Data to N2K
// =====================
void NMEA2000_SendWindData(tNMEA2000_STM32 *N2k)
{
  tN2kMsg N2kMsg;

  if (WindScheduler.IsTime()) {
    WindScheduler.UpdateNextTime();

    // Get current wind data
    float WindSpeed, WindAngle;
    bool valid;
    GetWindData(&WindSpeed, &WindAngle, &valid);

    // Only send if we have valid data
    if (valid) {
      // PGN 130306: Wind Data
      // Parameters: SID, WindSpeed (m/s), WindAngle (radians), WindReference
      // Apparent wind: angle 0° = bow, increases clockwise (0–360°).
      SetN2kWindSpeed(N2kMsg, windSID, WindSpeed, DegToRad(WindAngle), N2kWind_Apprent);
      if (N2k->SendMsg(N2kMsg)) {
        LED2_Flash();  // Brief flash on each successful TX
      }

      // Advance SID (0-252 per NMEA 2000 spec; 253-255 are reserved)
      windSID++;
      if (windSID > 252) windSID = 0;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
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
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
