#include <Arduino.h>

// UART Bridge for STM32 Filament Sensor
// Connect Nucleo TX (D1) -> Mega RX1 (Pin 19)
// Connect Nucleo GND -> Mega GND

void setup() {
  // Initialize USB Serial connection to PC
  Serial.begin(115200);
  
  // Initialize Hardware Serial1 for Nucleo connection
  Serial1.begin(115200);

  Serial.println("Elegoo Mega 2560 UART Bridge Initialized!");
  Serial.println("Listening for STM32 data on RX1 (Pin 19)...");
  Serial.println("=========================================");
}

void loop() {
  // Forward data from the STM32 (Serial1) to the PC Terminal (Serial)
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }

  // Forward any user input from the PC Terminal back to the STM32 (optional)
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
  }
}