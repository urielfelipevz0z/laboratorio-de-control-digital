#include <Arduino.h>

constexpr double KP = 0.48, KI = 0.055, KD = 8E-7; // Nota: KI y KD reducidos para escala ADC
constexpr double TI = 1.0 / KI, TD = 1.0 / KD; /** TODO: verificar definiciones de TD y TI */
const double KR = 1.0 / sqrt(TI*TD); // Ganancia de rastreo/antiwindup
constexpr int16_t MAX_OUT_VALUE = 255;

constexpr int CONTROL_MISO_PIN = 5;
constexpr int CONTROL_MOSI_PIN = 6;
constexpr unsigned long SAMPLE_PERIOD = 10; /** TODO: ver el maximo valor adecuado */
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ = 5000;     // 5kHz - mucho más rápido que actualización
constexpr int PWM_RESOLUTION = 8;  // 8 bits para simplicidad (0-255)

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

  long output = (long)(KP*error) + (long)(KI*integral) + (long)(KD*derivative);

  bool windup = (output > MAX_OUT_VALUE || output < 0);
  if (windup) {
    // Si hay windup, reiniciar integral y derivada
    integral = 0;
    previous_error = 0;
    if (antiwindup) {
      const long antiwindup_feedback = (long)(KR * (output - MAX_OUT_VALUE));
      output = (long)(KP*error) +
               (long)((KI + antiwindup_feedback)*integral) +
               (long)(KD*derivative);
    }
    output = (output > MAX_OUT_VALUE) ? MAX_OUT_VALUE : 0; // Limitar a 0 si es negativo
  }

  return output;
}

void setup() {
  Serial.begin(9600);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(CONTROL_MOSI_PIN, PWM_CHANNEL);
}

void loop() {
  // Ejemplo de uso con valores ADC directos
  unsigned long current_millis = millis();
  static unsigned long last_millis = 0;
  const int16_t ref = (sin(2 * PI * 0.5 * (current_millis / 1000.0)) > 0.0) * (0.5 * MAX_OUT_VALUE);
  if (current_millis - last_millis >= SAMPLE_PERIOD) {
    last_millis = current_millis;
    Serial.printf("Referencia: %d\n", ref);
    const int16_t actual = analogRead(CONTROL_MISO_PIN);
    const int16_t error =  ref - actual;

    const int16_t control_output = pid_controller(error);
    //const int16_t control_output = ref;
    ledcWrite(PWM_CHANNEL, control_output);
    Serial.printf("Actual: %d, Error: %d, Control Output: %d\n", actual, error, control_output);
  }
}