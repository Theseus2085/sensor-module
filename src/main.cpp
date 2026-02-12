/**
 * @file main.cpp
 * @brief STM32F446RE Filament Width Sensor Module (mbed OS Framework)
 * @description Dual Hall effect sensor module for 3D printer filament diameter monitoring
 * 
 * Hardware:
 * - STM32F446RE Nucleo Board
 * - 2x Hall Effect Sensors (SS495A) on PA0/PA1 (ADC inputs)
 * - I2C1 Slave on PB8/PB9 for printer communication
 * - USB Serial for debug output at 115200 baud
 * - Calibration buttons on PB6/PB7
 * 
 * I2C Connection to Printer:
 * - SCL: PB8 - Connect to printer's I2C SCL
 * - SDA: PB9 - Connect to printer's I2C SDA  
 * - GND: Common ground required
 * 
 * Framework: mbed OS
 */

#include "mbed.h"

// ============================================================================
// FIRMWARE / DEBUG CONFIGURATION
// ============================================================================

#define FW_VERSION "0.5.1-dbg.addrlog.1"

#ifndef I2C_DEBUG_ENABLE
  #if defined(NDEBUG)
    #define I2C_DEBUG_ENABLE 0
  #else
    #define I2C_DEBUG_ENABLE 1
  #endif
#endif

#ifndef I2C_DEBUG_PRINT_PERIOD_MS
  #define I2C_DEBUG_PRINT_PERIOD_MS 1000
#endif

#ifndef I2C_DEBUG_EVENT_QUEUE_LEN
  #define I2C_DEBUG_EVENT_QUEUE_LEN 64
#endif

#if I2C_DEBUG_EVENT_QUEUE_LEN < 2
  #error "I2C_DEBUG_EVENT_QUEUE_LEN must be at least 2"
#endif

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// mbed I2CSlave::address expects the 8-bit form.
// Keep this paired with printer `FILWIDTH_SENSOR_I2C_ADDRESS`:
// addr8 = (addr7 << 1), e.g. 0x42 -> 0x84.
#define SENSOR_I2C_ADDRESS 0x84
#define SENSOR_I2C_FREQUENCY_HZ 400000

// ADC pins for Hall effect sensors
AnalogIn sensor1(PA_0);  // ADC1_IN0
AnalogIn sensor2(PA_1);  // ADC1_IN1

// I2C slave
I2CSlave i2c_slave(PB_9, PB_8);  // SDA, SCL

// Digital outputs/inputs
DigitalOut led(PA_5);
DigitalIn cal_start_btn(PB_6, PullUp);
DigitalIn cal_next_btn(PB_7, PullUp);

// Serial debug
BufferedSerial pc(USBTX, USBRX, 115200);

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

/* Test Mode */
#define TEST_MODE 0

#if TEST_MODE
  #define TEST_SENSOR1_MM 1.99f
  #define TEST_SENSOR2_MM 1.99f
#endif

/* ADC Averaging Buffer */
#define ADC_BUFFER_SIZE 64
#define ADC_BUFFER_MASK (ADC_BUFFER_SIZE - 1)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

/* Sensor Measurements */
volatile float sensor1_mm = 1.75f;
volatile float sensor2_mm = 1.75f;

/* Calibration Tables */
struct CalibrationPoint {
  uint16_t raw_adc;
  float diameter_mm;
};

CalibrationPoint calibration_tables[2][3] = {
  { // Sensor 1
    {7, 1.47f},
    {532, 1.68f},
    {1119, 1.99f}
  },
  { // Sensor 2
    {7, 1.47f},
    {532, 1.68f},
    {1119, 1.99f}
  }
};

/* ADC Circular Buffers for Averaging */
uint16_t adc_buffer_sensor1[ADC_BUFFER_SIZE];
uint16_t adc_buffer_sensor2[ADC_BUFFER_SIZE];
uint16_t adc_buffer_index = 0;
bool adc_buffer_filled = false;

/* Spike Rejection - Last Valid Readings */
uint16_t last_valid_raw1 = 532; 
uint16_t last_valid_raw2 = 532;

/* I2C Communication Buffer */
volatile uint8_t tx_buffer[10] = {0};

/* I2C Connection Status */
volatile uint32_t i2c_request_count = 0;
volatile uint64_t last_i2c_request_time_us = 0;

enum I2cDebugEventType : uint8_t {
  EV_READ_ADDRESSED = 1,
  EV_WRITE_ADDRESSED = 2,
  EV_WRITE_GENERAL = 3,
  EV_WRITE_READ_ERROR = 4,
  EV_SLAVE_REINIT = 5,
};

struct I2cDebugEvent {
  uint8_t type;
};

struct I2cDebugCounters {
  uint32_t total_events;
  uint32_t read_addressed;
  uint32_t write_addressed;
  uint32_t write_general;
  uint32_t write_read_error;
  uint32_t slave_reinit;
  uint32_t queue_overflow;
};

#if I2C_DEBUG_ENABLE
volatile I2cDebugEvent i2c_debug_event_queue[I2C_DEBUG_EVENT_QUEUE_LEN];
volatile uint16_t i2c_debug_event_head = 0;
volatile uint16_t i2c_debug_event_tail = 0;
volatile uint32_t i2c_debug_event_overflow = 0;
I2cDebugCounters i2c_debug_counters = {};
#endif

/* Timing */
Timer heartbeat_timer;
Timer serial_timer;
Timer i2c_check_timer;
Timer i2c_debug_timer;
Timer uptime_timer;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void format_sensor_data(float val, uint8_t* buf);
void reinit_i2c_slave();
void enqueue_i2c_debug_event(I2cDebugEventType event_type);
bool dequeue_i2c_debug_event(I2cDebugEvent &event);
void consume_i2c_debug_events();
void print_i2c_debug_info();
void print_mm_value(const char* label, float value_mm);
uint64_t get_uptime_us();

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

uint16_t read_sensor_raw_adc(uint8_t sensor_idx) {
  AnalogIn* sensor_pin = (sensor_idx == 0) ? &sensor1 : &sensor2;

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

  const CalibrationPoint* table = calibration_tables[sensor_idx];
  float diameter;

  if (raw_adc <= table[1].raw_adc) {
    int32_t denom = (int32_t)table[1].raw_adc - (int32_t)table[0].raw_adc;
    if (denom == 0) {
      diameter = table[0].diameter_mm;
    } else {
      float slope = (table[1].diameter_mm - table[0].diameter_mm) / (float)denom;
      diameter = table[0].diameter_mm + slope * (float)((int32_t)raw_adc - (int32_t)table[0].raw_adc);
    }
  } else {
    int32_t denom = (int32_t)table[2].raw_adc - (int32_t)table[1].raw_adc;
    if (denom == 0) {
      diameter = table[1].diameter_mm;
    } else {
      float slope = (table[2].diameter_mm - table[1].diameter_mm) / (float)denom;
      diameter = table[1].diameter_mm + slope * (float)((int32_t)raw_adc - (int32_t)table[1].raw_adc);
    }
  }
  
  return diameter;
}

void measure_sensor_values(void) {
  uint16_t raw1 = read_sensor_raw_adc(0);
  uint16_t raw2 = read_sensor_raw_adc(1);
  
  last_valid_raw1 = raw1;
  last_valid_raw2 = raw2;
  
  adc_buffer_sensor1[adc_buffer_index] = last_valid_raw1;
  adc_buffer_sensor2[adc_buffer_index] = last_valid_raw2;
  
  adc_buffer_index = (adc_buffer_index + 1) & ADC_BUFFER_MASK;
  
  if (adc_buffer_index == 0 && !adc_buffer_filled) {
    adc_buffer_filled = true;
  }
  
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  uint16_t samples_to_average = adc_buffer_filled ? ADC_BUFFER_SIZE : adc_buffer_index;
  
  if (samples_to_average == 0) {
    samples_to_average = 1;
  }
  
  for (uint16_t i = 0; i < samples_to_average; i++) {
    sum1 += adc_buffer_sensor1[i];
    sum2 += adc_buffer_sensor2[i];
  }
  
  uint16_t avg_raw1 = (uint16_t)(sum1 / samples_to_average);
  uint16_t avg_raw2 = (uint16_t)(sum2 / samples_to_average);
  
  sensor1_mm = convert_raw_adc_to_mm(avg_raw1, 0);
  sensor2_mm = convert_raw_adc_to_mm(avg_raw2, 1);
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
  format_sensor_data(1.75f, (uint8_t*)&tx_buffer[0]);
  format_sensor_data(1.75f, (uint8_t*)&tx_buffer[5]);

  for (int s = 0; s < 2; s++) {
    printf("Calibrating Sensor %d\n", s + 1);
    
    for (int p = 0; p < 3; p++) {
      printf("  S%d Point %d (%.2fmm) - Press NEXT button...\n", s + 1, p + 1, diameters[p]);

      // Wait for button press
      while(cal_next_btn.read() == 1) {
        ThisThread::sleep_for(10ms);
        measure_sensor_values();
      }
      ThisThread::sleep_for(50ms);

      // Capture calibration point
      calibration_tables[s][p].raw_adc = read_sensor_raw_adc(s);
      calibration_tables[s][p].diameter_mm = diameters[p];
      
      printf("    Captured ADC: %u\n", calibration_tables[s][p].raw_adc);

      // Wait for button release
      while(cal_next_btn.read() == 0) {
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

void format_sensor_data(float val, uint8_t* buf) {
  if (val < 0.0f) val = 0.0f;
  if (val > 9.9999f) val = 9.9999f;

  uint32_t int_val = (uint32_t)(val * 10000.0f + 0.5f);
  
  buf[0] = (int_val / 10000) % 10;
  buf[1] = (int_val / 1000) % 10;
  buf[2] = (int_val / 100) % 10;
  buf[3] = (int_val / 10) % 10;
  buf[4] = (int_val / 1) % 10;
}

void enqueue_i2c_debug_event(I2cDebugEventType event_type) {
#if I2C_DEBUG_ENABLE
  core_util_critical_section_enter();

  const uint16_t next_head = (uint16_t)((i2c_debug_event_head + 1) % I2C_DEBUG_EVENT_QUEUE_LEN);
  if (next_head == i2c_debug_event_tail) {
    // Drop newest event if queue is full to preserve non-blocking behavior.
    i2c_debug_event_overflow++;
    core_util_critical_section_exit();
    return;
  }

  i2c_debug_event_queue[i2c_debug_event_head].type = (uint8_t)event_type;
  i2c_debug_event_head = next_head;

  core_util_critical_section_exit();
#else
  (void)event_type;
#endif
}

bool dequeue_i2c_debug_event(I2cDebugEvent &event) {
#if I2C_DEBUG_ENABLE
  bool has_data = false;

  core_util_critical_section_enter();

  if (i2c_debug_event_tail != i2c_debug_event_head) {
    event.type = i2c_debug_event_queue[i2c_debug_event_tail].type;
    i2c_debug_event_tail = (uint16_t)((i2c_debug_event_tail + 1) % I2C_DEBUG_EVENT_QUEUE_LEN);
    has_data = true;
  }

  core_util_critical_section_exit();

  return has_data;
#else
  (void)event;
  return false;
#endif
}

void consume_i2c_debug_events() {
#if I2C_DEBUG_ENABLE
  I2cDebugEvent event = {};

  while (dequeue_i2c_debug_event(event)) {
    i2c_debug_counters.total_events++;

    switch ((I2cDebugEventType)event.type) {
      case EV_READ_ADDRESSED:
        i2c_debug_counters.read_addressed++;
        break;
      case EV_WRITE_ADDRESSED:
        i2c_debug_counters.write_addressed++;
        break;
      case EV_WRITE_GENERAL:
        i2c_debug_counters.write_general++;
        break;
      case EV_WRITE_READ_ERROR:
        i2c_debug_counters.write_read_error++;
        break;
      case EV_SLAVE_REINIT:
        i2c_debug_counters.slave_reinit++;
        break;
      default:
        break;
    }
  }

  i2c_debug_counters.queue_overflow = i2c_debug_event_overflow;
#endif
}

uint64_t get_uptime_us() {
  return (uint64_t)uptime_timer.elapsed_time().count();
}

void print_mm_value(const char* label, float value_mm) {
  if (value_mm < 0.0f) value_mm = 0.0f;

  const uint32_t milli = (uint32_t)(value_mm * 1000.0f + 0.5f);
  printf("%s: %lu.%03lumm", label, (unsigned long)(milli / 1000), (unsigned long)(milli % 1000));
}

void print_i2c_debug_info() {
#if I2C_DEBUG_ENABLE
  if (i2c_debug_timer.elapsed_time() < std::chrono::milliseconds(I2C_DEBUG_PRINT_PERIOD_MS)) {
    return;
  }

  i2c_debug_timer.reset();

  printf("I2CDBG own7=0x%02X total=%lu rd=%lu wr=%lu gc=%lu ioerr=%lu reinits=%lu qovf=%lu req=%lu \n",
         SENSOR_I2C_ADDRESS >> 1,
         (unsigned long)i2c_debug_counters.total_events,
         (unsigned long)i2c_debug_counters.read_addressed,
         (unsigned long)i2c_debug_counters.write_addressed,
         (unsigned long)i2c_debug_counters.write_general,
         (unsigned long)i2c_debug_counters.write_read_error,
         (unsigned long)i2c_debug_counters.slave_reinit,
         (unsigned long)i2c_debug_counters.queue_overflow,
         (unsigned long)i2c_request_count);
#endif
}

void print_debug_info(const char* status) {
  if (serial_timer.elapsed_time() >= 5s) {
    serial_timer.reset();
    
    uint64_t now_us = get_uptime_us();
    bool i2c_active = i2c_request_count > 0 && (now_us - last_i2c_request_time_us) < 5000000ULL;
    const char* i2c_status = i2c_active ? "ACTIVE" : "IDLE";
    
#if TEST_MODE
    print_mm_value("[TEST MODE] S1", TEST_SENSOR1_MM);
    printf(" | ");
    print_mm_value("S2", TEST_SENSOR2_MM);
#else
    print_mm_value("S1", sensor1_mm);
    printf(" | ");
    print_mm_value("S2", sensor2_mm);
#endif
    
    printf(" | ADC: [%u, %u]", last_valid_raw1, last_valid_raw2);
    printf(" | I2C: %s (%lu requests) | %s\n", i2c_status, (unsigned long)i2c_request_count, status);
  }
}

void check_i2c_connection() {
  if (i2c_check_timer.elapsed_time() >= 10s) {
    i2c_check_timer.reset();
    
    uint64_t now_us = get_uptime_us();
    bool i2c_active = i2c_request_count > 0 && (now_us - last_i2c_request_time_us) < 5000000ULL;
    
    if (!i2c_active && i2c_request_count == 0) {
      printf("I2C: Waiting for printer master connection...\n");
    } else if (!i2c_active && i2c_request_count > 0) {
      printf("I2C: Master disconnected\n");
    }
  }
}

void reinit_i2c_slave() {
  i2c_slave.stop();
  i2c_slave.frequency(SENSOR_I2C_FREQUENCY_HZ);
  i2c_slave.address(SENSOR_I2C_ADDRESS);
  enqueue_i2c_debug_event(EV_SLAVE_REINIT);
}

// ============================================================================
// I2C SLAVE THREAD
// ============================================================================

void i2c_slave_thread() {
  while (true) {
    int status = i2c_slave.receive();

    if (status == I2CSlave::NoData) {
      // Small delay avoids starving lower-priority threads when the bus is idle.
      ThisThread::sleep_for(1ms);
      continue;
    }

    if (status == I2CSlave::WriteGeneral) {
      enqueue_i2c_debug_event(EV_WRITE_GENERAL);
      char dummy;
      if (i2c_slave.read(&dummy, 1) != 0) {
        enqueue_i2c_debug_event(EV_WRITE_READ_ERROR);
        reinit_i2c_slave();
        continue;
      }
    }
    else if (status == I2CSlave::WriteAddressed) {
      // Master is sending data (not used in this application)
      enqueue_i2c_debug_event(EV_WRITE_ADDRESSED);
      char dummy;
      if (i2c_slave.read(&dummy, 1) != 0) {
        enqueue_i2c_debug_event(EV_WRITE_READ_ERROR);
        reinit_i2c_slave();
        continue;
      }
    }
    else if (status == I2CSlave::ReadAddressed) {
      // Master is requesting data - send the 10-byte buffer immediately
      enqueue_i2c_debug_event(EV_READ_ADDRESSED);
      i2c_request_count++;
      last_i2c_request_time_us = get_uptime_us();

      // Send directly from volatile buffer - it's always ready
      if (i2c_slave.write((const char *)tx_buffer, 10) != 0) {
        enqueue_i2c_debug_event(EV_WRITE_READ_ERROR);
        reinit_i2c_slave();
        continue;
      }
    }

    // Respond quickly while still allowing other threads to run.
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
#if I2C_DEBUG_ENABLE
  printf("I2C debug: ENABLED (period=%dms, queue=%d)\n", (int)I2C_DEBUG_PRINT_PERIOD_MS, (int)I2C_DEBUG_EVENT_QUEUE_LEN);
#else
  printf("I2C debug: DISABLED\n");
#endif
  printf("I2C note: unmatched_addr_not_visible_in_slave_mode=1\n");
  
  // Pre-fill I2C buffer with safe data FIRST
  sensor1_mm = 1.75f;
  sensor2_mm = 1.75f;
  format_sensor_data(1.75f, (uint8_t*)&tx_buffer[0]);
  format_sensor_data(1.75f, (uint8_t*)&tx_buffer[5]);
  
  // Quick buffer fill with real ADC data
  uint16_t r1 = (uint16_t)(sensor1.read() * 4095.0f);
  uint16_t r2 = (uint16_t)(sensor2.read() * 4095.0f);
  last_valid_raw1 = r1;
  last_valid_raw2 = r2;
  
  for(int i=0; i<ADC_BUFFER_SIZE; i++) {
     adc_buffer_sensor1[i] = r1;
     adc_buffer_sensor2[i] = r2;
  }
  adc_buffer_filled = true;
  
  // Update I2C buffer with initial real measurements
  measure_sensor_values();
  format_sensor_data(sensor1_mm, (uint8_t*)&tx_buffer[0]);
  format_sensor_data(sensor2_mm, (uint8_t*)&tx_buffer[5]);
  
  // Start timers
  heartbeat_timer.start();
  serial_timer.start();
  i2c_check_timer.start();
  i2c_debug_timer.start();
  uptime_timer.start();
  
  printf("Data ready. Starting I2C slave...\n");
  
  // CRITICAL: Initialize I2C slave LAST after all data is ready
  reinit_i2c_slave();
  
  // Start I2C slave thread - data is already prepared
  Thread i2c_thread(osPriorityRealtime);
  i2c_thread.start(i2c_slave_thread);
  printf("I2C thread started\n");
  
  // Start independent LED heartbeat thread
  Thread led_thread(osPriorityNormal);
  led_thread.start(led_heartbeat_thread);
  printf("LED thread starting...\n");
  
  // Small delay to let threads initialize
  ThisThread::sleep_for(200ms);
  
  printf("Ready!\n");
  
  while (true) {
    // Check for calibration button
    if (cal_start_btn.read() == 0) {
      ThisThread::sleep_for(50ms);  // Debounce
      if (cal_start_btn.read() == 0) {
        calibration();
        // Wait for button release
        while(cal_start_btn.read() == 0) {
          ThisThread::sleep_for(10ms);
        }
        ThisThread::sleep_for(50ms);
      }
    }
    
    // Update sensor measurements and I2C buffer
    measure_sensor_values();
    
    // Update I2C buffer atomically
    uint8_t temp_buf[10];
#if TEST_MODE
    format_sensor_data(TEST_SENSOR1_MM, temp_buf);
    format_sensor_data(TEST_SENSOR2_MM, temp_buf + 5);
#else
    format_sensor_data(sensor1_mm, temp_buf);
    format_sensor_data(sensor2_mm, temp_buf + 5);
#endif
    
    __disable_irq();
    memcpy((void*)tx_buffer, temp_buf, 10);
    __enable_irq();
    
    // Check I2C connection status
    check_i2c_connection();

    // Consume queued I2C events outside of the I2C thread and print summary periodically.
    consume_i2c_debug_events();
    print_i2c_debug_info();
    
    // Print debug info
    print_debug_info("Normal Mode");
    
    ThisThread::sleep_for(2ms);
  }
}
