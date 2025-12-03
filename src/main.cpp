/**
 * @file main.cpp
 * @brief STM32F446RE main application file using STM32Cube HAL
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>

/* 
 * TODO: Set this address to match FILWIDTH_SENSOR_I2C_ADDRESS in your Marlin configuration.
 * Note: HAL expects the address shifted left by 1.
 */
#define SENSOR_I2C_ADDRESS 0x42 

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

// Dummy sensor values - In a real application, read these from ADC/Sensor
float sensor1_mm = 1.75f;
float sensor2_mm = 1.75f;

struct CalibrationPoint {
  float voltage;
  float diameter_mm;
};

// Calibration tables for 2 sensors, each with 3 points (Low, Nominal, High)
// TODO: Update these values with actual calibration data for each sensor
CalibrationPoint calibration_tables[2][3] = {
  { // Sensor 1 Table
    {1.0f, 1.50f},  // Point 1
    {1.65f, 1.75f}, // Point 2
    {2.3f, 2.00f}   // Point 3
  },
  { // Sensor 2 Table
    {1.0f, 1.50f},  // Point 1
    {1.65f, 1.75f}, // Point 2
    {2.3f, 2.00f}   // Point 3
  }
};

#define CAL_START_BUTTON_PIN GPIO_PIN_4
#define CAL_START_BUTTON_PORT GPIOB
#define CAL_NEXT_BUTTON_PIN GPIO_PIN_5
#define CAL_NEXT_BUTTON_PORT GPIOB

float read_sensor_voltage(uint8_t sensor_idx) {
  ADC_ChannelConfTypeDef sConfig = {0};
  float voltage = 0.0f;

  if (sensor_idx == 0) {
    sConfig.Channel = ADC_CHANNEL_0;
  } else {
    sConfig.Channel = ADC_CHANNEL_1;
  }
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    uint32_t rawValue = HAL_ADC_GetValue(&hadc1);
    voltage = (float)rawValue * 3.3f / 4095.0f;
  }
  HAL_ADC_Stop(&hadc1);
  
  return voltage;
}

void calibration() {
  float diameters[] = {1.50f, 1.75f, 2.00f};
  
  // Loop through both sensors
  for (int s = 0; s < 2; s++) {
    // Loop through 3 calibration points
    for (int p = 0; p < 3; p++) {
      // Wait for Next Button Press (Active Low)
      while(HAL_GPIO_ReadPin(CAL_NEXT_BUTTON_PORT, CAL_NEXT_BUTTON_PIN) == GPIO_PIN_SET) {
        HAL_Delay(10);
      }
      HAL_Delay(50); // Debounce press

      // Record value
      calibration_tables[s][p].voltage = read_sensor_voltage(s);
      calibration_tables[s][p].diameter_mm = diameters[p];

      // Wait for Next Button Release
      while(HAL_GPIO_ReadPin(CAL_NEXT_BUTTON_PORT, CAL_NEXT_BUTTON_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(10);
      }
      HAL_Delay(50); // Debounce release
    }
  }
}

float convert_voltage_to_mm(float voltage, uint8_t sensor_idx) {
  if (sensor_idx >= 2) return 1.75f; // Safety check

  const CalibrationPoint* table = calibration_tables[sensor_idx];

  // Linear interpolation between points
  // Assumes table is sorted by voltage
  
  if (voltage <= table[1].voltage) {
    // Interpolate between Point 1 and Point 2
    float denom = table[1].voltage - table[0].voltage;
    if (fabs(denom) < 0.0001f) return table[0].diameter_mm;
    
    float slope = (table[1].diameter_mm - table[0].diameter_mm) / denom;
    return table[0].diameter_mm + slope * (voltage - table[0].voltage);
  } else {
    // Interpolate between Point 2 and Point 3
    float denom = table[2].voltage - table[1].voltage;
    if (fabs(denom) < 0.0001f) return table[1].diameter_mm;

    float slope = (table[2].diameter_mm - table[1].diameter_mm) / denom;
    return table[1].diameter_mm + slope * (voltage - table[1].voltage);
  }
}

void measure_sensor_values() {
  float voltage;

  // --- Read Sensor 1 (Channel 0 / PA0) ---
  voltage = read_sensor_voltage(0);
  sensor1_mm = convert_voltage_to_mm(voltage, 0);

  // --- Read Sensor 2 (Channel 1 / PA1) ---
  voltage = read_sensor_voltage(1);
  sensor2_mm = convert_voltage_to_mm(voltage, 1);
}

// Buffer to send to printer: 2 axes * 5 bytes = 10 bytes
uint8_t tx_buffer[10];

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void Error_Handler(void);

/**
 * @brief Formats a float diameter into the 5-byte format expected by the printer.
 * Format: X.XXXX (implied decimal)
 * Bytes are raw values 0-9.
 */
void format_sensor_data(float val, uint8_t* buf) {
  // Clamp value to reasonable range (0.0 - 9.9999)
  if (val < 0.0f) val = 0.0f;
  if (val > 9.9999f) val = 9.9999f;

  // Convert to fixed point integer (X.XXXX -> XXXXX)
  uint32_t int_val = (uint32_t)(val * 10000.0f + 0.5f); // +0.5 for rounding

  // Extract digits. 
  // The printer decoder reads: result = digits[0] + digits[1]*0.1 + ...
  // So buf[0] is the integer part.
  
  buf[0] = (int_val / 10000) % 10;
  buf[1] = (int_val / 1000) % 10;
  buf[2] = (int_val / 100) % 10;
  buf[3] = (int_val / 10) % 10;
  buf[4] = (int_val / 1) % 10;
}

void update_buffer() {
  format_sensor_data(sensor1_mm, &tx_buffer[0]);
  format_sensor_data(sensor2_mm, &tx_buffer[5]);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();

  while (1)
  {
    // Check for Calibration Start Button (Active Low)
    if (HAL_GPIO_ReadPin(CAL_START_BUTTON_PORT, CAL_START_BUTTON_PIN) == GPIO_PIN_RESET) {
      // Debounce
      HAL_Delay(50);
      if (HAL_GPIO_ReadPin(CAL_START_BUTTON_PORT, CAL_START_BUTTON_PIN) == GPIO_PIN_RESET) {
        calibration();
      }
    }

    // 1. Update sensor readings
    measure_sensor_values();
    
    // 2. Prepare the buffer
    update_buffer();

    // 3. Wait for I2C request from Printer
    // This is a blocking call with a timeout. 
    // In a production environment, consider using Interrupts (HAL_I2C_Slave_Transmit_IT)
    // or DMA to avoid blocking the sensor reading loop.
    if (HAL_I2C_Slave_Transmit(&hi2c1, tx_buffer, 10, 50) != HAL_OK) {
      // If error is not just a timeout (no master request), handle it
      if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
         // Reset I2C or handle error
         // __HAL_I2C_CLEAR_FLAG(&hi2c1, ...);
      }
    }
    
    // Small delay to prevent tight loop if no I2C activity
    // HAL_Delay(1); 
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = SENSOR_I2C_ADDRESS << 1; // Shifted for HAL
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
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
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param adcHandle: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Initializes the RCC Oscillators according to the specified parameters
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  // User can add their own implementation to report the HAL error return state
  __disable_irq();
  while (1)
  {
    // Stay here in case of error
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  // User can add their own implementation to report the file name and line number,
  // for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif /* USE_FULL_ASSERT */
