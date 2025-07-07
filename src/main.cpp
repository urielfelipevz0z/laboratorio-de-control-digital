#include <Arduino.h>

#include "PidController.hxx"
#include "globals.hxx"

/**
 * TODO: incluir ecuación de diferencias para hacer simulaciones sin el motor.
 * TODO: Hacer que la recopilación de muestras se detenga cuando hemos llegado al
 *       estado estable para Takahashi-Chan-Auslander y optimización genética
 * TODO: hacer que el código en todas partes trabaje en segundos y no
 *       milisegundos
 * TODO: Hacer que MISO sea filtrado por un filtro con $f_c = 50 \text{Hz}$
 */

void setup() {
  Serial.begin(115200);
  pinMode(pin::MISO, INPUT);
  pinMode(pin::MOSI, OUTPUT);

  analogReadResolution(10); // Para compatibilidad con Arduino UNO

  analogWriteFrequency(pin::MOSI, 500);
  analogWriteResolution(pin::MOSI, 8);
}

void loop() {
  static PidController pid;

  millis_t current = millis();
  static millis_t last = 0;
  if (current - last >= timing::SAMPLE_PERIOD) {
    const int16_t ref = (sin(2*PI * 0.5*(current / 1000.0)) > 0) * (0.5 * MAX_OUT_VALUE);
    last = current;
    const int16_t actual = analogRead(pin::MISO);
    const int16_t error =  ref - actual;

    const int16_t control_output = pid(error);
    analogWrite(pin::MOSI, control_output);
    Serial.printf("ref: %d, vel: %d, e: %d, pid: %d\n", ref, actual, error, control_output);
  }
}