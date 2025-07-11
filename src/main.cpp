#include <Arduino.h>
#include <cmath>

#include "PidController.hxx"
#include "globals.hxx"
#include "filter.hxx"
#include "OnOffController.hxx"

#ifndef PI
#define PI M_PI
#endif

/**
 * \brief Función para representar la ecuación de diferencias del modulo de motor para simular su comportamiento.
 * 
 * $$y(k) = 0.7967y(k - 1) + 0.1891r(k - 1)$$
 * 
 * \param input Entrada de la ec. en dif. siendo exactamente $r(k - 1)$
 * \return La respuesta del motor al instante $k$ siendo $y(k)$
 */

int16_t plant_model(u16_t input) {
  double realInput = static_cast<double>(input);
  realInput = realInput*(3.3/4095); // Scale input to 12-bit range
  static double y_prev = 0.0;
  double r_prev = static_cast<double>(input);
  double y = 0.7967 * y_prev + 0.1891 * r_prev;
  y_prev = y;
  int16_t y_scaled = static_cast<int16_t>(y * (4095 / 3.3));
  return y_scaled;
}

filter::ExpLowPassFilter sensor_filter(40.0);

// PID with conservative gains for stability
PidController pid(1.0, 0.0, 0.0);
OnOffController on_off(0.1); // Hysteresis of 0.1 for on/off control

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
    const int16_t ref = (sin(2*PI * 0.5*(current / 1000.0)) > 0) * (0.3 *MAX_OUT_VALUE);
    last = current;
    
    ledcWrite(1, ref); // Send reference to channel 1 for monitoring
    
    // Read sensor with proper filtering and scaling
    const int16_t actual_raw = analogRead(pin::MISO);
    const int16_t actual_filtered = sensor_filter(actual_raw);
    const int16_t actual = (actual_filtered * MAX_OUT_VALUE) / 4095; // Scale 12-bit to 8-bit
    //static int16_t prev_output = 0;
    //const int16_t actual = plant_model(prev_output);
    const int16_t error = ref - actual;

    // Get PID output
    //const int16_t control_output = pid(error);
    // Get On/Off control output
    const int16_t control_output = on_off(error);
    //prev_output = control_output;
    
    ledcWrite(0, control_output); // Send control signal to motor
    Serial.printf("%15d, %15d, %15d, %15d\n", ref, actual_filtered, control_output, error);
    //Serial.printf("%d, %d, %d, %d\n", ref, actual, control_output, error);

    last = current;
  }
}
