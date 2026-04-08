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

#include "KVStore.h"
#include "kvstore_global_api.h"
#include "mbed.h"

#define CAL_KV_KEY "/kv/cal_tables"

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
DigitalIn log_btn(PB_5, PullUp);      // Arduino D4 (Logging Toggle)

// Mega 2560 UART Bridge
// TX mapped to PB_10 (Arduino header D6)
BufferedSerial mega_uart(PB_10, NC, 115200);

// Override mbed OS default console so all standard printf() calls route to D6
namespace mbed {
FileHandle *mbed_override_console(int fd) { return &mega_uart; }
} // namespace mbed

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
volatile uint16_t burst_second_sample[2] = {0, 0};

/* Calibration Tables */
struct CalibrationPoint {
  uint16_t raw_adc;
  float diameter_mm;
};

CalibrationPoint calibration_tables[2][3] = {{// Sensor 1
                                              {401, 1.65f},
                                              {656, 1.75f},
                                              {867, 1.85f}},
                                             {// Sensor 2
                                              {703, 1.65f},
                                              {900, 1.75f},
                                              {1063, 1.85f}}};

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
void load_calibration_from_flash();

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

void load_calibration_from_flash() {
  size_t actual_size = 0;
  int ret = kv_get(CAL_KV_KEY, (void *)calibration_tables,
                   sizeof(calibration_tables), &actual_size);
  if (ret == MBED_SUCCESS && actual_size == sizeof(calibration_tables)) {
    printf("Loaded persistent calibration from internal flash (size: %u)\n",
           actual_size);
  } else {
    printf("No persistent calibration found, using standard defaults.\n");
  }

  // Dump active calibration table in copy-pasteable C format
  printf("\n--- Active Calibration Table (copy-paste into source) ---\n");
  printf("CalibrationPoint calibration_tables[2][3] = {\n");
  for (int s = 0; s < 2; s++) {
    printf("  { // Sensor %d\n", s + 1);
    for (int p = 0; p < 3; p++) {
      int d_int = (int)calibration_tables[s][p].diameter_mm;
      int d_frac =
          (int)((calibration_tables[s][p].diameter_mm - d_int) * 100 + 0.5f);
      printf("    {%u, %d.%02df}%s\n", calibration_tables[s][p].raw_adc, d_int,
             d_frac, (p < 2) ? "," : "");
    }
    printf("  }%s\n", (s < 1) ? "," : "");
  }
  printf("};\n");
  printf("--- End Calibration Table ---\n\n");
}

uint16_t read_sensor_raw_adc(uint8_t sensor_idx) {
  AnalogIn *sensor_pin = (sensor_idx == 0) ? &sensor1 : &sensor2;

  // Oversample with 32-sample burst (12-bit ADC)
  const int burstCount = 32;
  int32_t burstSum = 0;
  for (int k = 0; k < burstCount; k++) {
    uint16_t val = (uint16_t)(sensor_pin->read() * 4095.0f);
    if (k == 1) { // 2nd sample in the burst (index 1)
      burst_second_sample[sensor_idx] = val;
    }
    burstSum += val;
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

  float diameters[] = {1.65f, 1.75f, 1.85f};

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

      // Capture calibration point (deep temporal averaging of 32 bursts spread
      // over 160ms)
      uint32_t cal_sum = 0;
      for (int i = 0; i < 32; i++) {
        cal_sum += read_sensor_raw_adc(s);
        ThisThread::sleep_for(5ms);
      }
      calibration_tables[s][p].raw_adc = (uint16_t)(cal_sum / 32);
      calibration_tables[s][p].diameter_mm = diameters[p];

      printf("    Captured ADC: %u\n", calibration_tables[s][p].raw_adc);

      // Wait for button release
      while (cal_next_btn.read() == 0) {
        ThisThread::sleep_for(10ms);
      }
      ThisThread::sleep_for(50ms);
    }
  }

  // Save the new table securely to non-volatile flash memory
  int ret = kv_set(CAL_KV_KEY, (const void *)calibration_tables,
                   sizeof(calibration_tables), 0);
  if (ret == MBED_SUCCESS) {
    printf("Calibration saved securely to Internal Flash!\n");
  } else {
    printf("Warning: Failed to save to Internal Flash (error %d)\n", ret);
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
  printf("UART test stream thread started (Outputting exactly 1 event to D6 "
         "per D4 Button Press!)\n");
  char buffer[128];
  uint32_t index = 1;

  while (true) {
    // Wait for the button to be pressed
    if (log_btn.read() == 0) {
      // Debounce the press
      ThisThread::sleep_for(20ms);
      if (log_btn.read() == 0) {

        // Current averaged MM values
        int s1_int = (int)sensor1_mm;
        int s1_frac = (int)((sensor1_mm - s1_int) * 10000);
        int s2_int = (int)sensor2_mm;
        int s2_frac = (int)((sensor2_mm - s2_int) * 10000);

        // Compute single sample MM values for the 2nd raw value of the burst
        float single_s1 = convert_raw_adc_to_mm(burst_second_sample[0], 0);
        int ss1_int = (int)single_s1;
        int ss1_frac = (int)((single_s1 - ss1_int) * 10000);

        float single_s2 = convert_raw_adc_to_mm(burst_second_sample[1], 1);
        int ss2_int = (int)single_s2;
        int ss2_frac = (int)((single_s2 - ss2_int) * 10000);

        int len = snprintf(buffer, sizeof(buffer),
                           "%lu,%d.%04d,%d.%04d,%d.%04d,%d.%04d\n", index++,
                           s1_int, s1_frac, s2_int, s2_frac, ss1_int, ss1_frac,
                           ss2_int, ss2_frac);

        // Output directly over the hardware UART to the Mega
        if (len > 0) {
          mega_uart.write(buffer, len);
        }

        printf("Measurement %lu captured!\n", index - 1);

        // Wait for release so it strictly only captures ONE event per press
        while (log_btn.read() == 0) {
          ThisThread::sleep_for(10ms);
        }
        ThisThread::sleep_for(50ms); // Debounce the physical release
      }
    }

    // Idle loop waiting for next button press
    ThisThread::sleep_for(10ms);
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

  // >> LOAD CALIBRATION <<
  load_calibration_from_flash();

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
