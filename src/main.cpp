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
// I2C2 removed - using Software I2C for OLED on D6/D3
ADC_HandleTypeDef hadc1;

// Dummy sensor values - In a real application, read these from ADC/Sensor
float sensor1_mm = 1.75f;
float sensor2_mm = 1.75f;

// Variable to track if I2C transmission is active/pending
volatile uint8_t i2c_tx_busy = 0;


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

// --- OLED (0.96" SSD1306 assumed) on separate I2C peripheral ---
// NOTE: Default pins for I2C2 on STM32F446: PB10=SCL, PB11=SDA (AF4)
// Update these if your wiring differs.
#define OLED_I2C_ADDRESS (0x3C << 1) // HAL expects 7-bit address shifted left by 1
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// --- Software I2C Configuration for OLED on D6 (SCL) and D3 (SDA) ---
// D6 = PB10 (SCL)
// D3 = PB3  (SDA)
#define SW_I2C_SCL_PIN GPIO_PIN_10
#define SW_I2C_SCL_PORT GPIOB
#define SW_I2C_SDA_PIN GPIO_PIN_3
#define SW_I2C_SDA_PORT GPIOB

static void sw_i2c_delay(void) {
  // Simple delay for ~100kHz-400kHz. Adjust based on clock speed.
  // For 180MHz, a loop is needed.
  for(volatile int i=0; i<50; i++) { __NOP(); }
}

static void sw_i2c_scl_hi(void) {
  HAL_GPIO_WritePin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, GPIO_PIN_SET);
}

static void sw_i2c_scl_lo(void) {
  HAL_GPIO_WritePin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, GPIO_PIN_RESET);
}

static void sw_i2c_sda_hi(void) {
  HAL_GPIO_WritePin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, GPIO_PIN_SET);
}

static void sw_i2c_sda_lo(void) {
  HAL_GPIO_WritePin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, GPIO_PIN_RESET);
}

static void sw_i2c_start(void) {
  sw_i2c_sda_hi();
  sw_i2c_scl_hi();
  sw_i2c_delay();
  sw_i2c_sda_lo();
  sw_i2c_delay();
  sw_i2c_scl_lo();
}

static void sw_i2c_stop(void) {
  sw_i2c_sda_lo();
  sw_i2c_scl_hi();
  sw_i2c_delay();
  sw_i2c_sda_hi();
  sw_i2c_delay();
}

static void sw_i2c_write_byte(uint8_t byte) {
  for(uint8_t i=0; i<8; i++) {
    if(byte & 0x80) sw_i2c_sda_hi();
    else sw_i2c_sda_lo();
    sw_i2c_delay();
    sw_i2c_scl_hi(); // Clock High
    sw_i2c_delay();
    sw_i2c_scl_lo(); // Clock Low
    byte <<= 1;
  }
  
  // ACK / NACK Clock Pulse
  sw_i2c_sda_hi(); // Release SDA for ACK
  sw_i2c_delay();
  sw_i2c_scl_hi();
  sw_i2c_delay();
  // We ignore the ACK value for OLED simplicity
  sw_i2c_scl_lo();
}

static uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];
static uint8_t oled_cursor_x = 0;
static uint8_t oled_cursor_page = 0;

static void oled_send_command(uint8_t cmd) {
  sw_i2c_start();
  sw_i2c_write_byte(OLED_I2C_ADDRESS);
  sw_i2c_write_byte(0x00); // Command stream
  sw_i2c_write_byte(cmd);
  sw_i2c_stop();
}

static void oled_send_data(const uint8_t* data, uint16_t len) {
  // Try to send efficiently in one transaction if possible, or chunks
  sw_i2c_start();
  sw_i2c_write_byte(OLED_I2C_ADDRESS);
  sw_i2c_write_byte(0x40); // Data stream
  
  for(uint16_t i=0; i<len; i++) {
      sw_i2c_write_byte(data[i]);
  }
  
  sw_i2c_stop();
}

static void oled_init(void) {
  HAL_Delay(50);

  oled_send_command(0xAE); // Display OFF
  oled_send_command(0xD5); // Set display clock
  oled_send_command(0x80);
  oled_send_command(0xA8); // Set multiplex
  oled_send_command(0x3F); // 64
  oled_send_command(0xD3); // Set display offset
  oled_send_command(0x00);
  oled_send_command(0x40); // Set start line
  oled_send_command(0x8D); // Charge pump
  oled_send_command(0x14);
  oled_send_command(0x20); // Memory mode
  oled_send_command(0x00); // Horizontal addressing
  oled_send_command(0xA1); // Segment remap
  oled_send_command(0xC8); // COM scan direction
  oled_send_command(0xDA); // COM pins
  oled_send_command(0x12);
  oled_send_command(0x81); // Contrast
  oled_send_command(0x7F);
  oled_send_command(0xD9); // Pre-charge
  oled_send_command(0xF1);
  oled_send_command(0xDB); // VCOM detect
  oled_send_command(0x40);
  oled_send_command(0xA4); // Display follows RAM
  oled_send_command(0xA6); // Normal display
  oled_send_command(0x2E); // Deactivate scroll
  oled_send_command(0xAF); // Display ON

  memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

static void oled_set_cursor(uint8_t x, uint8_t page) {
  if (x >= OLED_WIDTH) x = 0;
  if (page >= (OLED_HEIGHT / 8)) page = 0;
  oled_cursor_x = x;
  oled_cursor_page = page;
}

static void oled_clear(void) {
  memset(oled_buffer, 0x00, sizeof(oled_buffer));
  oled_set_cursor(0, 0);
}

static void oled_update(void) {
  for (uint8_t page = 0; page < (OLED_HEIGHT / 8); page++) {
    oled_send_command((uint8_t)(0xB0 + page));
    oled_send_command(0x00);
    oled_send_command(0x10);
    oled_send_data(&oled_buffer[OLED_WIDTH * page], OLED_WIDTH);
  }
}

// Minimal 5x7-ish glyphs (columns) for: '0'..'9', 'S', '.', ' ' only.
// LSB is top pixel within the 8-pixel page.
static const uint8_t glyph_digits[10][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};

static const uint8_t glyph_S[5] = {0x26, 0x49, 0x49, 0x49, 0x32};
static const uint8_t glyph_dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};

static void oled_draw_glyph_5x7(const uint8_t glyph[5]) {
  if (oled_cursor_page >= (OLED_HEIGHT / 8)) return;
  if (oled_cursor_x + 6 > OLED_WIDTH) return;

  uint16_t idx = (uint16_t)oled_cursor_page * OLED_WIDTH + oled_cursor_x;
  for (uint8_t col = 0; col < 5; col++) {
    oled_buffer[idx + col] = glyph[col];
  }
  oled_buffer[idx + 5] = 0x00; // 1-column spacing
  oled_cursor_x = (uint8_t)(oled_cursor_x + 6);
}

static void oled_write_char(char c) {
  if (c >= '0' && c <= '9') {
    oled_draw_glyph_5x7(glyph_digits[c - '0']);
    return;
  }
  if (c == 'S') {
    oled_draw_glyph_5x7(glyph_S);
    return;
  }
  if (c == '.') {
    oled_draw_glyph_5x7(glyph_dot);
    return;
  }
  // space or unsupported
  static const uint8_t glyph_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  oled_draw_glyph_5x7(glyph_space);
}

static void oled_write_string(const char* s) {
  while (*s) {
    oled_write_char(*s++);
  }
}

static void format_mm_3dp(float val, char out[8]) {
  // Produces "X.XXX" (up to 9.999), always 5 chars + NUL.
  if (val < 0.0f) val = 0.0f;
  if (val > 9.999f) val = 9.999f;
  uint32_t fp = (uint32_t)(val * 1000.0f + 0.5f);
  uint32_t i = fp / 1000;
  uint32_t f = fp % 1000;
  out[0] = (char)('0' + (i % 10));
  out[1] = '.';
  out[2] = (char)('0' + ((f / 100) % 10));
  out[3] = (char)('0' + ((f / 10) % 10));
  out[4] = (char)('0' + (f % 10));
  out[5] = '\0';
}

static void oled_show_normal(void) {
  char buf[8];
  oled_clear();

  oled_set_cursor(0, 0);
  oled_write_string("S1 ");
  format_mm_3dp(sensor1_mm, buf);
  oled_write_string(buf);

  oled_set_cursor(0, 2);
  oled_write_string("S2 ");
  format_mm_3dp(sensor2_mm, buf);
  oled_write_string(buf);

  oled_update();
}

static void oled_show_cal_prompt(uint8_t sensor_idx, float ref_mm) {
  char buf[8];
  oled_clear();

  oled_set_cursor(0, 0);
  oled_write_string("S");
  oled_write_char((char)('1' + sensor_idx));

  oled_set_cursor(0, 2);
  format_mm_3dp(ref_mm, buf);
  oled_write_string(buf);

  oled_update();
}

static volatile uint8_t g_calibration_mode = 0;

// Forward declaration (used before the main function prototypes block)
void Error_Handler(void);

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

  g_calibration_mode = 1;
  
  // Loop through both sensors
  for (int s = 0; s < 2; s++) {
    // Loop through 3 calibration points
    for (int p = 0; p < 3; p++) {
      // Wait for Next Button Press (Active Low)

      oled_show_cal_prompt((uint8_t)s, diameters[p]);

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

  g_calibration_mode = 0;
  oled_show_normal();
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

// I2C Interrupt Handlers
extern "C" {
  void I2C1_EV_IRQHandler(void);
  void I2C1_ER_IRQHandler(void);
}

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

  oled_init();
  oled_show_normal();

  // Start I2C Slave listening (Interrupt mode)
  if (HAL_I2C_Slave_Transmit_IT(&hi2c1, tx_buffer, 10) != HAL_OK) {
      Error_Handler();
  }

  uint32_t last_oled_update = HAL_GetTick();

  while (1)
  {
    // Check for Calibration Start Button (Active Low)
    if (HAL_GPIO_ReadPin(CAL_START_BUTTON_PORT, CAL_START_BUTTON_PIN) == GPIO_PIN_RESET) {
      // Debounce
      HAL_Delay(50);
      if (HAL_GPIO_ReadPin(CAL_START_BUTTON_PORT, CAL_START_BUTTON_PIN) == GPIO_PIN_RESET) {
        calibration();
        last_oled_update = HAL_GetTick();
      }
    }

    // 1. Update sensor readings
    measure_sensor_values();
    
    // 2. Prepare the buffer
    update_buffer();

    // Update OLED every 10 seconds in normal mode
    if (!g_calibration_mode) {
      uint32_t now = HAL_GetTick();
      if ((now - last_oled_update) >= 1000U) {
        oled_show_normal();
        last_oled_update = now;
      }
    }
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
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Enable Interrupts
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

// Re-arm the I2C Slave TX after completion
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        HAL_I2C_Slave_Transmit_IT(&hi2c1, tx_buffer, 10);
    }
}

// Handle errors (like NACK or Bus Error) by re-arming
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Clear flags and restart
        // __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BUSY); // Dangerous to force clear
        // Simply trying to restart listening is often enough
        HAL_I2C_Slave_Transmit_IT(&hi2c1, tx_buffer, 10);
    }
}

// Interrupt Dispatchers
void I2C1_EV_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void) {
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

// Removed MX_I2C2_Init as we use Software I2C on GPIOs

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance == I2C1) 
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL (Arduino D15)
    PB9     ------> I2C1_SDA (Arduino D14)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
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

  /* Configure SW I2C Pins: D6 (PB10) and D3 (PB3) as Output Open Drain */
  GPIO_InitStruct.Pin = SW_I2C_SCL_PIN | SW_I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Enable internal pullups for convenience
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Set initial high state (Idle)
  HAL_GPIO_WritePin(GPIOB, SW_I2C_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, SW_I2C_SDA_PIN, GPIO_PIN_SET);

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
