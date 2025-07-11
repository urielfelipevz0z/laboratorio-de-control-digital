#include <Arduino.h>
#include <cmath>

#include "PidController.hxx"
#include "globals.hxx"
#include "filter.hxx"

#ifndef PI
#define PI M_PI
#endif

filter::ExpLowPassFilter sensor_filter(40.0);

// PID with conservative gains for stability
PidController pid(5.0, 100.0, 1e-3);

void setup() {
  Serial.begin(115200);
  pinMode(pin::MISO, INPUT);
  pinMode(pin::MOSI, OUTPUT);
  pinMode(15, OUTPUT);

  analogReadResolution(12); // Use ESP32-S3's full 12-bit ADC resolution

  ledcSetup(1, 500, 8); // Canal 1, frecuencia de 500 Hz, resolución de 8 bits
  ledcAttachPin(15, 1); // Asignar el pin 15 al canal 1
  ledcSetup(0, 500, 8); // Canal 0, frecuencia de 500 Hz, resolución de 8 bits
  ledcAttachPin(pin::MOSI, 0); // Asignar el pin MOSI al canal 0
}

void loop() {
  static uint32_t last = 0;
  uint32_t current = millis();
  
  if (current - last >= timing::SAMPLE_PERIOD) {
    const int16_t ref = (sin(2*PI * 0.5*(current / 1000.0)) > 0) * (0.5 * MAX_OUT_VALUE);
    last = current;
    
    ledcWrite(1, ref); // Send reference to channel 1 for monitoring
    
    // Read sensor with proper filtering and scaling
    const int16_t actual_raw = analogRead(pin::MISO);
    const double actual_filtered = sensor_filter(actual_raw);
    const int16_t actual = (actual_filtered * MAX_OUT_VALUE) / 4095; // Scale 12-bit to 8-bit
    const int16_t error = ref - actual;

    // Get PID output
    const int16_t control_output = pid(error);
    const int16_t control_bounded = constrain(control_output, 0, MAX_OUT_VALUE);
    
    ledcWrite(0, control_bounded); // Send control signal to motor
    Serial.printf("ref: %d, actual: %d, error: %d, output: %d\n", ref, actual, error, control_bounded);
    
    last = current;
  }
}
