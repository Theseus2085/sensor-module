/**
 * @file main.cpp
 * @brief STM32F446RE Filament Width Sensor Module (mbed OS Framework)
 * @description Dual Hall effect sensor module for 3D printer filament diameter
 * monitoring
 *
 * Hardware:
 * - STM32F446RE Nucleo Board
 * - 2x Hall Effect Sensors (SS495A) on PA0/PA1 (ADC inputs)
 * - I2C1 Slave on PB8/PB9 for printer communication
 * - USB Serial at 115200 baud
 * - Calibration buttons on PB6/D10 (Start) and PA9/D8 (Next)
 *
 * I2C Connection to Printer:
 * - SCL: PB8 - Connect to printer's I2C SCL D7
 * - SDA: PB9 - Connect to printer's I2C SDA D8
 * - GND: Common ground required
 *
 * Framework: mbed OS
 */

#include "mbed.h"

// ============================================================================
// FIRMWARE CONFIGURATION
// ============================================================================

#define FW_VERSION "0.6.0"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// mbed I2CSlave::address expects the 8-bit form.
// Keep this paired with printer `FILWIDTH_SENSOR_I2C_ADDRESS`:
// addr8 = (addr7 << 1), e.g. 0x42 -> 0x84.
#define SENSOR_I2C_ADDRESS 0x84
#define SENSOR_I2C_FREQUENCY_HZ 400000

// ADC pins for Hall effect sensors
AnalogIn sensor1(PA_0); // ADC1_IN0
AnalogIn sensor2(PA_1); // ADC1_IN1

// I2C slave
I2CSlave i2c_slave(PB_9, PB_8); // SDA, SCL

// Digital outputs/inputs
DigitalOut led(PA_5);
DigitalIn cal_start_btn(PB_6, PullUp);
DigitalIn cal_next_btn(PA_9, PullUp); // Arduino D8

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

/* Test Mode */
#define TEST_MODE 0

#if TEST_MODE
#define TEST_SENSOR1_MM 1.99f
#define TEST_SENSOR2_MM 1.99f
#define TEST_SENSOR1_X10000 ((uint32_t)(TEST_SENSOR1_MM * 10000.0f + 0.5f))
#define TEST_SENSOR2_X10000 ((uint32_t)(TEST_SENSOR2_MM * 10000.0f + 0.5f))
#endif

#define SENSOR_MM_FIXED_SCALE 10000U
#define SENSOR_MM_FIXED_MAX 99999U

/* Sensor Measurements */
volatile float sensor1_mm = 1.75f;
volatile float sensor2_mm = 1.75f;

/* Calibration Tables */
struct CalibrationPoint {
  uint16_t raw_adc;
  float diameter_mm;
};

CalibrationPoint calibration_tables[2][3] = {{// Sensor 1
                                              {7, 1.47f},
                                              {532, 1.68f},
                                              {1119, 1.99f}},
                                             {// Sensor 2
                                              {7, 1.47f},
                                              {532, 1.68f},
                                              {1119, 1.99f}}};

/* I2C Communication Buffer */
volatile uint8_t tx_buffer[10] = {0};

/* I2C Connection Status */
volatile uint32_t i2c_request_count = 0;
volatile uint64_t last_i2c_request_time_us = 0;

/* Timing */
Timer heartbeat_timer;
Timer uptime_timer;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

uint32_t mm_to_fixed_10000(float val);
void format_sensor_data_fixed(uint32_t val_x10000, uint8_t *buf);
void reinit_i2c_slave();
uint64_t get_uptime_us();

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

uint16_t read_sensor_raw_adc(uint8_t sensor_idx) {
  AnalogIn *sensor_pin = (sensor_idx == 0) ? &sensor1 : &sensor2;

  // Oversample with 16-sample burst (12-bit ADC)
  const int burstCount = 16;
  int32_t burstSum = 0;
  for (int k = 0; k < burstCount; k++) {
    burstSum += (uint16_t)(sensor_pin->read() * 4095.0f);
  }

  uint16_t averaged = (uint16_t)(burstSum / burstCount);
  return averaged;
}

float convert_raw_adc_to_mm(uint16_t raw_adc, uint8_t sensor_idx) {
  if (sensor_idx >= 2) {
    return 1.75f;
  }

  const CalibrationPoint *table = calibration_tables[sensor_idx];
  float diameter;

  if (raw_adc <= table[1].raw_adc) {
    int32_t denom = (int32_t)table[1].raw_adc - (int32_t)table[0].raw_adc;
    if (denom == 0) {
      diameter = table[0].diameter_mm;
    } else {
      float slope =
          (table[1].diameter_mm - table[0].diameter_mm) / (float)denom;
      diameter = table[0].diameter_mm +
                 slope * (float)((int32_t)raw_adc - (int32_t)table[0].raw_adc);
    }
  } else {
    int32_t denom = (int32_t)table[2].raw_adc - (int32_t)table[1].raw_adc;
    if (denom == 0) {
      diameter = table[1].diameter_mm;
    } else {
      float slope =
          (table[2].diameter_mm - table[1].diameter_mm) / (float)denom;
      diameter = table[1].diameter_mm +
                 slope * (float)((int32_t)raw_adc - (int32_t)table[1].raw_adc);
    }
  }

  return diameter;
}

void measure_sensor_values(void) {
  uint16_t raw1 = read_sensor_raw_adc(0);
  uint16_t raw2 = read_sensor_raw_adc(1);

  sensor1_mm = convert_raw_adc_to_mm(raw1, 0);
  sensor2_mm = convert_raw_adc_to_mm(raw2, 1);
}

// ============================================================================
// CALIBRATION
// ============================================================================

void calibration(void) {
  printf("\n=== Calibration Started ===\n");

  float diameters[] = {1.50f, 1.75f, 2.00f};

  // Pre-fill buffer output to safe 1.75mm
  sensor1_mm = 1.75f;
  sensor2_mm = 1.75f;
  format_sensor_data_fixed(mm_to_fixed_10000(1.75f), (uint8_t *)&tx_buffer[0]);
  format_sensor_data_fixed(mm_to_fixed_10000(1.75f), (uint8_t *)&tx_buffer[5]);

  for (int s = 0; s < 2; s++) {
    printf("Calibrating Sensor %d\n", s + 1);

    for (int p = 0; p < 3; p++) {
      printf("  S%d Point %d (%.2fmm) - Press NEXT button...\n", s + 1, p + 1,
             diameters[p]);

      // Wait for button press
      while (cal_next_btn.read() == 1) {
        ThisThread::sleep_for(10ms);
        measure_sensor_values();
      }
      ThisThread::sleep_for(50ms);

      // Capture calibration point
      calibration_tables[s][p].raw_adc = read_sensor_raw_adc(s);
      calibration_tables[s][p].diameter_mm = diameters[p];

      printf("    Captured ADC: %u\n", calibration_tables[s][p].raw_adc);

      // Wait for button release
      while (cal_next_btn.read() == 0) {
        ThisThread::sleep_for(10ms);
      }
      ThisThread::sleep_for(50ms);
    }
  }

  printf("=== Calibration Complete ===\n\n");
}

// ============================================================================
// COMMUNICATION HELPERS
// ============================================================================

uint32_t mm_to_fixed_10000(float val) {
  if (val < 0.0f)
    val = 0.0f;
  if (val > 9.9999f)
    val = 9.9999f;
  return (uint32_t)(val * (float)SENSOR_MM_FIXED_SCALE + 0.5f);
}

void format_sensor_data_fixed(uint32_t val_x10000, uint8_t *buf) {
  if (val_x10000 > SENSOR_MM_FIXED_MAX)
    val_x10000 = SENSOR_MM_FIXED_MAX;

  buf[0] = (val_x10000 / 10000U) % 10U;
  buf[1] = (val_x10000 / 1000U) % 10U;
  buf[2] = (val_x10000 / 100U) % 10U;
  buf[3] = (val_x10000 / 10U) % 10U;
  buf[4] = val_x10000 % 10U;
}

uint64_t get_uptime_us() {
  // Timer::elapsed_time() reports microseconds on mbed chrono durations.
  return (uint64_t)uptime_timer.elapsed_time().count();
}

void reinit_i2c_slave() {
  i2c_slave.stop();
  // mbed reinitializes the peripheral in frequency(); set address afterwards.
  i2c_slave.frequency(SENSOR_I2C_FREQUENCY_HZ);
  i2c_slave.address(SENSOR_I2C_ADDRESS);
}

// ============================================================================
// I2C SLAVE THREAD
// ============================================================================

void i2c_slave_thread() {
  while (true) {
    int status = i2c_slave.receive();

    if (status == I2CSlave::NoData) {
      // Yield CPU while idle without adding latency to addressed transactions.
      ThisThread::sleep_for(1ms);
      continue;
    }

    if (status == I2CSlave::WriteGeneral) {
      // General-call writes are ignored (best-effort drain for compatibility).
      char dummy;
      (void)i2c_slave.read(&dummy, 1);
    } else if (status == I2CSlave::WriteAddressed) {
      // Host write probes are treated as non-fatal (best-effort drain).
      char dummy;
      (void)i2c_slave.read(&dummy, 1);
    } else if (status == I2CSlave::ReadAddressed) {
      // Primary path: respond with the latest 10-byte diameter payload.
      i2c_request_count++;
      last_i2c_request_time_us = get_uptime_us();

      // In test mode, serve fixed test payload directly on each read.
#if TEST_MODE
      uint8_t test_payload[10];
      format_sensor_data_fixed(TEST_SENSOR1_X10000, test_payload);
      format_sensor_data_fixed(TEST_SENSOR2_X10000, test_payload + 5);
      if (i2c_slave.write((const char *)test_payload, 10) != 0) {
#else
      // Buffer is continuously refreshed by main loop; no copy/allocation here.
      if (i2c_slave.write((const char *)tx_buffer, 10) != 0) {
#endif
        printf("I2C: read-response write failed, reinitializing slave\n");
        reinit_i2c_slave();
        continue;
      }
    }

    // Keep bus service latency low while cooperating with other threads.
    ThisThread::yield();
  }
}

// ============================================================================
// LED HEARTBEAT THREAD (Independent - blinks even if main loop freezes)
// ============================================================================

void led_heartbeat_thread() {
  printf("LED thread started\n");
  while (true) {
    led = 1;
    ThisThread::sleep_for(200ms);
    led = 0;
    ThisThread::sleep_for(200ms);
  }
}

// ============================================================================
// UART TEST STREAM THREAD (Outputs measurements every 250ms)
// ============================================================================

void uart_stream_thread() {
  printf("UART test stream thread started\n");
  while (true) {
    // Output comma-separated values: Sensor1, Sensor2
    printf("%.4f,%.4f\n", sensor1_mm, sensor2_mm);
    ThisThread::sleep_for(250ms);
  }
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main() {
  // LED on during init
  led = 1;

  printf("\n=== STM32 Sensor (mbed OS) ===\n");
  printf("FW: %s\n", FW_VERSION);
  printf("I/O: 3.3V (matches Prusa MK4)\n");
  printf("I2C: 400kHz Fast Mode\n");
  printf("Address7: 0x%02X\n", SENSOR_I2C_ADDRESS >> 1);
  printf("Address8: 0x%02X\n", SENSOR_I2C_ADDRESS);

#if TEST_MODE
  sensor1_mm = TEST_SENSOR1_MM;
  sensor2_mm = TEST_SENSOR2_MM;
  printf("TEST_MODE active: direct fixed I2C payload (%.4f, %.4f)\n",
         TEST_SENSOR1_MM, TEST_SENSOR2_MM);
#else
  // Pre-fill I2C buffer with safe data FIRST
  sensor1_mm = 1.75f;
  sensor2_mm = 1.75f;
  format_sensor_data_fixed(mm_to_fixed_10000(1.75f), (uint8_t *)&tx_buffer[0]);
  format_sensor_data_fixed(mm_to_fixed_10000(1.75f), (uint8_t *)&tx_buffer[5]);

  // Initial measurement with real ADC data
  measure_sensor_values();
  format_sensor_data_fixed(mm_to_fixed_10000(sensor1_mm),
                           (uint8_t *)&tx_buffer[0]);
  format_sensor_data_fixed(mm_to_fixed_10000(sensor2_mm),
                           (uint8_t *)&tx_buffer[5]);
#endif

  // Start timers
  heartbeat_timer.start();
  uptime_timer.start();

  printf("Data ready. Starting I2C slave...\n");

  // Bring up the slave after payload initialization to avoid serving stale
  // bytes.
  reinit_i2c_slave();

  // Start I2C slave thread - data is already prepared
  Thread i2c_thread(osPriorityRealtime);
  i2c_thread.start(i2c_slave_thread);
  printf("I2C thread started\n");

  // Start independent LED heartbeat thread
  Thread led_thread(osPriorityNormal);
  led_thread.start(led_heartbeat_thread);
  printf("LED thread starting...\n");

  // Start UART test stream thread for Mega 2560 bridge
  Thread uart_thread(osPriorityNormal);
  uart_thread.start(uart_stream_thread);
  printf("UART test thread starting...\n");

  // Small delay to let threads initialize
  ThisThread::sleep_for(200ms);

  printf("Ready!\n");

  while (true) {
    // Check for calibration button
    if (cal_start_btn.read() == 0) {
      ThisThread::sleep_for(50ms); // Debounce
      if (cal_start_btn.read() == 0) {
        calibration();
        // Wait for button release
        while (cal_start_btn.read() == 0) {
          ThisThread::sleep_for(10ms);
        }
        ThisThread::sleep_for(50ms);
      }
    }

    // Update sensor measurements and I2C buffer
#if !TEST_MODE
    measure_sensor_values();

    // Update I2C buffer atomically
    uint8_t temp_buf[10];
    format_sensor_data_fixed(mm_to_fixed_10000(sensor1_mm), temp_buf);
    format_sensor_data_fixed(mm_to_fixed_10000(sensor2_mm), temp_buf + 5);

    __disable_irq();
    memcpy((void *)tx_buffer, temp_buf, 10);
    __enable_irq();
#endif

    ThisThread::sleep_for(2ms);
  }
}
