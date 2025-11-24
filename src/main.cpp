/**
 * @file main.cpp
 * @brief STM32F446RE main application file using STM32Cube HAL
 */

#include "stm32f4xx_hal.h"
#include <string.h>

/* 
 * TODO: Set this address to match FILWIDTH_SENSOR_I2C_ADDRESS in your Marlin configuration.
 * Note: HAL expects the address shifted left by 1.
 */
#define SENSOR_I2C_ADDRESS 0x40 

I2C_HandleTypeDef hi2c1;

// Dummy sensor values - In a real application, read these from ADC/Sensor
float sensor1_mm = 1.75f;
float sensor2_mm = 1.75f;

// Buffer to send to printer: 2 axes * 5 bytes = 10 bytes
uint8_t tx_buffer[10];

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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

  while (1)
  {
    // 1. Update sensor readings (Simulated here)
    // In real code: Read ADC, calculate width
    // sensor1_mm = read_sensor_1();
    
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
