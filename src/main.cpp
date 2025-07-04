#include <cmath>
#include <Arduino.h>

/**
 * TODO: incluir ecuación de diferencias para hacer simulaciones sin el motor.
 * TODO: Hacer que la recopilación de muestras se detenga cuando hemos llegado al
 *       estado estable para Takahashi-Chan-Auslander y optimización genética
 * TODO: hacer que el código en todas partes trabaje en segundos y no
 *       milisegundos
 */

using millis_t = unsigned long;
namespace pin {
  constexpr int MOTOR_MISO = 5;
  constexpr int MOTOR_MOSI = 6;
}

namespace timing {
  constexpr millis_t SAMPLE_PERIOD = 10; /** TODO: ver el maximo valor adecuado */
}
constexpr int16_t MAX_OUT_VALUE = 255;

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

};

/**
 * TODO: Generalizar este controlador a ON-OFF y P en la medida de lo posible.
 */
class pidController {
private:
  double k_p;  ///< Ganancia proporcional del PID
  double k_i;  ///< Ganancia integral del PID  
  double k_d;  ///< Ganancia derivativa del PID
  
  double T_i;  ///< Tiempo integral (Ti = ki/kp) - usado para cálculo de antiwindup
  double T_d;  ///< Tiempo derivativo (Td = kd/kp) - usado para cálculo de antiwindup
  double k_r;  ///< Ganancia de rastreo/antiwindup (kr = 1/√(Ti*Td))

  bool antiwindup = true;  ///< Habilita/deshabilita el mecanismo antiwindup

  /**
   * \brief Utiliza las reglas de Takahashi-Chan-Auslander para obtener las
   *        ganancias recomendadas para un PID.
   */
  static
  std::tuple<double, double, double> takahashi(double L, double R, double k)
  {
    double k_p, k_i, k_d;

    /**
     * Ganancia proporcional
     * 
     * Fórmula en LaTeX:
     *   $$ k_p = \frac{1.2}{R(L+h)} - \frac{k}{2} $$
     */
    k_p = ((1.2)/(R * (L + timing::SAMPLE_PERIOD/1000.0))) - (k/2);

    /**
     * Ganancia integral
     * 
     * Fórmula en LaTeX:
     *   $$ k_i = \frac{0.6 h}{R(L + \frac{h}{2})^2} $$
     */
    k_i = (0.6*timing::SAMPLE_PERIOD) /
                (R * std::pow((L + (timing::SAMPLE_PERIOD/1000.0)/2), 2));

    /**
     * Ganancia derivativa
     * 
     * Fórmula en LaTeX:
     *   $$ k_d = \frac{0.6}{R h} $$
     */
    k_d = 0.6 / (R * (timing::SAMPLE_PERIOD / 1000.0));

    return std::make_tuple(k_p, k_i, k_d);
  }

  /**
   * \brief Optimiza las ganancias del PID utilizando un algoritmo genético a
   *        partir de un conjunto de ganancias iniciales.
   * 
   * Este método toma ganancias que deberían de ser buenas para ser tomadas en
   * la población inicial, más adelante se ejecuta un algoritmo genético probando
   * la planta con un tiempo máximo de ejecución de un minuto. Probando cerca de
   * 6 individuos en 8 generaciones en los motores del laboratorio cuando mucho.
   * 
   * \note Si se llega a la convergencia antes del minuto esta función termina
   *       antes.
   * 
   * TODO: implementar algoritmo genético de optimización que pruebe el motor
   *       y determine al mejor de la población, en base al menor coste 
   *       (tiempo de respuesta + sobreimpulso)
   */
  static std::tuple<double, double, double>
  optimize_genetic(double k_p, double k_i, double k_d)
  {
    return std::make_tuple(k_p, k_i, k_d);
  }

  /**
   * \brief Realiza una derivada discreta sucia a partir de las muestras
   *        que va recibiendo continuamente.
   * 
   * Esta se trata de una ecuación en diferencias que representa un filtro
   * pasa bajas con $f_c = 50 \text{Hz}$ para evitar que la derivada se
   * dispare, y claro, realizamos la derivada después de filtrar la entrada.
   * Esto está representado en una ecuación de diferencias calculada de forma
   * analítica partiendo de $T=0.01$
   * $$ y(k) = 0.04321y(k-1) + 95.6786r(k) - 95.6786r(k-1) $$
   * 
   * TODO: Verificar que la ecuación de diferencias es correcta, en caso de
   *       que no usar la derivada sucia del profesor.
   * 
   * \return La pendiente actual sin ruidos
   */
  int16_t
  num_derivative(int16_t curr_in)
  {
    static int16_t last_y = 0;
    static int16_t last_in = 0;
    int16_t   y = 0.04321 * last_y + 95.6786 * curr_in - 95.6786 * last_in;
    last_y = y;
    last_in = curr_in;
    return y;
  }

public:
  /** TODO: poner los valores que encontremos nosotros manualmente*/
  pidController(double k_p = 0.03, double k_i = 0.6, double k_d = 0.6E-3)
    : k_p(k_p), k_i(k_i), k_d(k_d)
  {
    T_i = k_i / k_p;
    T_d = k_d / k_p;
    k_r = 1.0 / std::sqrt(T_i*T_d);
  }

  /**
   * \brief Sintoniza el controlador PID en base a la planta a la que está
   *        conetado.
   * 
   * Este método sintoniza el PID utilizando el método de
   * Takahashi-Chan-Auslander, obteniendo los parámetros L y R empíricamente
   * desde la planta, por lo que esta función es apta para sintonizar con
   * plantas de primer y segundo orden.
   * 
   * \note Esta función puede tomar su tiempo en ejecutarse, debido a la
   *       posible lentitud de la planta
   *
   */
  void
  tune()
  {
    
    const uint32_t total_test_samples = (millis_t)1000 / timing::SAMPLE_PERIOD; /*< Esperamos estabilización antes de 1 segundo en el peor caso*/
    int16_t samples[total_test_samples]; /*< Esperamos estabilización antes de 1 segundo en el peor caso*/

    const int16_t step_magnitude = 0.5 * MAX_OUT_VALUE; /*< Valor del escalón de prueba */
    int16_t steady_value_pre_step = analogRead(pin::MOTOR_MISO);
    analogWrite(pin::MOTOR_MOSI, step_magnitude); /*< Enviamos el escalón al motor */
    int16_t max_slope = 0;            /*< Máxima pendiente en la respuesta */
    std::pair<double, double> p1;     /*< El punto donde se encontró la máxima pendiente */
    int i = 0;
    for (; i < total_test_samples;) {
      static millis_t last = millis();
      if (millis() - last >= timing::SAMPLE_PERIOD) {
        
        samples[i] = analogRead(pin::MOTOR_MISO);

        if (i > 0) {
          int16_t slope = num_derivative(samples[i]);
          if (slope > max_slope) {
            max_slope = slope;
          }
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
    const double k = (steady_value_post_step - steady_value_pre_step) / step_magnitude;
    Rect R(p1, max_slope);
    double L = R.b / R.m; /*< Obtenemos el momento en que la recta cruza el eje Y */

    /**
     * Asignamos las ganancias del PID con el método de Takahashi-Chan-Auslander.
     * 
     * Estos valores serán más adelante ajustados con algún otro algoritmo
     * de optimización.
     */
    std::tie(k_p, k_i, k_d) = takahashi(L, R.m, k);

    std::tie(k_p, k_i, k_d) = optimize_genetic(k_p, k_i, k_d);

  }

  /**
   * \brief Obtiene la salida del controlador PID dado un error de entrada
   * \param error La diferencia entre el valor deseado y el actual 
   *        (setpoint - actual).
   * \return La señal del control generada por el PID, limitada al rango del
   *         ADC y lista para ser enviada a la planta.
   * 
   * \note Todos los valores entregados son considerados como voltajes
   *       positivos para la planta, por lo que este PID como tal no puede
   *       decirle a un motor que vaya hacia atrás.
   * 
   * TODO: ¿Cómo es que deberíamos manejar correcciones negativas del PID?
   *       El PID puede indicar ir en reversa pero nosotros como tal no
   *       podemos indicar eso.
   */
  int16_t
  operator()(int16_t error)
  {
    static long integral = 0;
    static int16_t previous_error = 0;

    integral += error;
    /** DUDA: ¿la derivada discreta requiere que restemos la diferencia de tiempos? */
    const int16_t derivative = (error - previous_error) / (timing::SAMPLE_PERIOD / 1000.0);
    previous_error = error;

    long control_signal = (long)(k_p*error) + (long)(k_i*integral) +
                  (long)(k_d*derivative);

    bool windup_ocurred = (control_signal > MAX_OUT_VALUE || control_signal < 0);
    if (windup_ocurred && antiwindup) {
      /* Este de aquí debería ser casi siempre negativo */
      const long antiwindup_feedback = (long)(k_r * (MAX_OUT_VALUE - control_signal));
      control_signal = (long)(k_p*error) +
               (long)((k_i + antiwindup_feedback)*integral) +
               (long)(k_d*derivative);
      /** DUDA: ¿Tenemos que hacer esto? No veo por qué */
      //control_signal = (control_signal > MAX_OUT_VALUE) ? MAX_OUT_VALUE : 0; // Limitar a 0 si es negativo
    }

    return control_signal;
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(pin::MOTOR_MISO, INPUT);
  pinMode(pin::MOTOR_MOSI, OUTPUT);

  analogReadResolution(10); // Para compatibilidad con Arduino UNO

  analogWriteFrequency(pin::MOTOR_MOSI, 500);
  analogWriteResolution(pin::MOTOR_MOSI, 8);
}

void loop() {
  static pidController pid;

  millis_t current = millis();
  static millis_t last = 0;
  if (current - last >= timing::SAMPLE_PERIOD) {
    const int16_t ref = (sin(2*PI * 0.5*(current / 1000.0)) > 0) * (0.5 * MAX_OUT_VALUE);
    last = current;
    const int16_t actual = analogRead(pin::MOTOR_MISO);
    const int16_t error =  ref - actual;

    const int16_t control_output = pid(error);
    analogWrite(pin::MOTOR_MOSI, control_output);
    Serial.printf("ref: %d, vel: %d, e: %d, pid: %d\n", ref, actual, error, control_output);
  }
}