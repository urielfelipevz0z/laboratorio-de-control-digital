#include <cmath>
#include <Arduino.h>

/**
 * TODO: hacer que el código en todas partes trabaje en segundos
 */
using millis_t = unsigned long;

struct Rect {
  double m;
  double b;


  
  public:

  /**
   * \brief Constructor para la clase Rect.
   * Hacemos una recta a partir de un punto y la pendiente.
   * \param m Pendiente de la recta.
   * \param p1 Un punto perteneciente a la recta.
   */
  Rect(std::pair<double, double> p1, double m)
  {
    this->m = m;
    this->b = p1.second - m * p1.first;
  }

}

/**
 * TODO: definir bien la clase del controlador PID
 * TODO: generalizar para los otros controladores (si se puede)
 */
class pidController {
  constexpr double KP = 0.03, KI = 0.6, KD = 0.6E-3; /** TODO: poner los valores que encontremos nosotros */
  constexpr double TI = KI / KP, TD = KD / KP;
  constexpr double KR = 1.0 / std::sqrt(TI*TD); /*< Ganancia de rastreo/antiwindup */

  void
  pid_tune()
  {
    double L; /*< Parámetros L y R en método de Takahashi-Chan-Auslander */
    int16_t max_slope; /*< Máxima pendiente de la recta */
    std::pair<double, double> p1; /*< Un punto perteneciente a la recta R */
    const uint32_t total_test_samples = (millis_t)1000 / time::SAMPLE_PERIOD; /*< Esperamos estabilización antes de 1 segundo en el peor caso*/
    int16_t samples[total_test_samples]; /*< Esperamos estabilización antes de 1 segundo en el peor caso*/

    int16_t ref = 0.5 * MAX_OUT_VALUE; /*< Valor del escalón de prueba */
    int16_t steady_value_pre_step = analogRead(pin::MOTOR_MISO);
    analogWrite(pin::MOTOR_MOSI, ref); /*< Enviamos el escalón al motor */
    int i = 0;
    for (; i < total_test_samples;) {
      static millis_t last = millis();
      if (millis() - last >= time::SAMPLE_PERIOD) {
        /**
         * TODO: detener la recopilación de muestras cuando hemos llegado al
         *       estado estable
         */
        samples[i] = analogRead(pin::MOTOR_MISO);

        if (i > 0) {
          /**
           * Precaución: esto puede dar resultados incorrectos si hay mucho ruido
           *             y cambios de alta frecuencia imprevistos en la señal.
           */
          max_slope = (samples[i] - samples[i-1] > max_slope)? samples[i] - samples[i-1] : max_slope;
          p1 = std::make_pair(i, samples[i]);
        }
        i += 1;
        last = millis();
      }
    }
    int16_t steady_value_post_step = samples[i - 1];
    analogWrite(pin::MOTOR_MOSI, 0); /*< Terminamos el escalón de prueba */

    /**
     * k es la ganancia en lazo abierto ante un escalón, según yo esta es la
     * listada como $K_I$ en el documento de la práctica 6. Evito nombrarla
     * así para evitar confusiones con la constante $k_i$ del PID.
     */
    const double k = (steady_value_post_step - steady_value_pre_step) / ref;
    Rect R(p1, max_slope);
    L = R.b / R.m; /*< Obtenemos el momento en que la recta cruza el eje Y */

    /**
     * Asignamos las ganancias del PID con el método de Takahashi-Chan-Auslander.
     * 
     * Estos valores serán más adelante ajustados con algún otro algoritmo
     * de optimización.
     */

    /**
     * Ganancia proporcional
     * 
     * Fórmula en LaTeX:
     *   $$ k_p = \frac{1.2}{R(L+h)} - \frac{k}{2} $$
     */
    this->k_p = ((1.2)/(R.m * (L + time::SAMPLE_PERIOD/1000.0))) - (k/2);

    /**
     * Ganancia integral
     * 
     * Fórmula en LaTeX:
     *   $$ k_i = \frac{0.6 h}{R(L + \frac{h}{2})^2} $$
     */
    this->k_i = (0.6*time::SAMPLE_PERIOD) /
                (R.m * std::pow((L + (time::SAMPLE_PERIOD/1000.0)/2), 2));

    /**
     * Ganancia derivativa
     * 
     * Fórmula en LaTeX:
     *   $$ k_d = \frac{0.6}{R h} $$
     */
    this->k_d = 0.6 / (R.m * (time::SAMPLE_PERIOD / 1000.0));
  }

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