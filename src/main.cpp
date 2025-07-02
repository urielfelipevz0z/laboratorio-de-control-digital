#include <cmath>
#include <Arduino.h>

using millis_t = unsigned long;

namespace ctrl {
  constexpr double KP = 0.03, KI = 0.6, KD = 0.6E-3; /** TODO: poner los valores que encontremos nosotros */
  constexpr double TI = KI / KP, TD = KD / KP;
  constexpr double KR = 1.0 / std::sqrt(TI*TD); /*< Ganancia de rastreo/antiwindup */ 
}


namespace pin {
  constexpr int MOTOR_MISO = 5;
  constexpr int MOTOR_MOSI = 6;
}

namespace time {
  constexpr millis_t SAMPLE_PERIOD = 10; /** TODO: ver el maximo valor adecuado */
}
constexpr int16_t MAX_OUT_VALUE = 255;

bool antiwindup;



/**
 * \brief Controlador PID simple para un motor DC.
 * \param error La diferencia entre el valor deseado y el actual (setpoint - actual).
 * \return La salida del controlador PID, limitada al rango del ADC.
 * \note No tomamos en consideracion que el motor pueda ir en reversa,
 *       todos los valores son para que vaya adelante.
 */
int16_t
pid_controller(int16_t error)
{
  static long integral = 0;
  static int16_t previous_error = 0;

  integral += error;
  const int16_t derivative = error - previous_error;
  previous_error = error;

  long output = (long)(ctrl::KP*error) + (long)(ctrl::KI*integral) + (long)(ctrl::KD*derivative);

  bool windup = (output > MAX_OUT_VALUE || output < 0);
  if (windup) {
    // Si hay windup, reiniciar integral y derivada
    integral = 0;
    previous_error = 0;
    if (antiwindup) {
      const long antiwindup_feedback = (long)(ctrl::KR * (MAX_OUT_VALUE - output));
      output = (long)(ctrl::KP*error) +
               (long)((ctrl::KI + antiwindup_feedback)*integral) +
               (long)(ctrl::KD*derivative);
    }
    output = (output > MAX_OUT_VALUE) ? MAX_OUT_VALUE : 0; // Limitar a 0 si es negativo
  }

  return output;
}

void setup() {
  Serial.begin(115200);
  pinMode(pin::MOTOR_MISO, INPUT);
  pinMode(pin::MOTOR_MOSI, OUTPUT);

  analogReadResolution(10); // Para compatibilidad con Arduino UNO

  analogWriteFrequency(pin::MOTOR_MOSI, 500);
  analogWriteResolution(pin::MOTOR_MOSI, 8);
}

void loop() {
  // Ejemplo de uso con valores ADC directos
  millis_t current = millis();
  static millis_t last = 0;
  if (current - last >= time::SAMPLE_PERIOD) {
    const int16_t ref = (sin(2*PI * 0.5*(current / 1000.0)) > 0) * (0.5 * MAX_OUT_VALUE);
    last = current;
    const int16_t actual = analogRead(pin::MOTOR_MISO);
    const int16_t error =  ref - actual;

    const int16_t control_output = pid_controller(error);
    analogWrite(pin::MOTOR_MOSI, control_output);
    Serial.printf("ref: %d, vel: %d, e: %d, pid: %d\n", ref, actual, error, control_output);
  }
}