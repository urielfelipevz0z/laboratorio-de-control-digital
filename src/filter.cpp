#include <Arduino.h>

#include "filter.hxx"
#include "globals.hxx"

/**
 * \brief Representa a un filtro pasa bajas exponencial de primer orden
 *        con respuesta en tiempo real.
 * 
 * En esencia, esta clase representa la ecuación en diferencias:
 *  $$ y(k) =  \alpha x(k) + (1-\alpha)y(k-1) $$
 */
class ExpLowPassFilter {
private:
  double alpha;             /**< Coeficiente $\alpha$ de suavizado  */
  double last_output;       /**< Última muestra suavizada $y(k-1)$ */

public:
  /**
   * \brief Construye el filtro obteniendo los valores adecuados para su
   *        funcionamiento.
   * 
   * Para obtener $\alpha$ usamos la fórmula:
   *   $$ \alpha = e^{-2\pi f_c T} $$
   * donde $f_c$ es la frecuencia de corte del filtro y $T$ es el tiempo de
   * muestreo.
   * 
   * \note Si la frecuencia de corte no es especificada, se tomará la máxima
   *       frecuencia reconstruible del sistema $f_N$.
   */
  ExpLowPassFilter(double cutoff_freq = timing::NYQUIST_FREQ):
   alpha(exp(- (2.0 * M_PI * cutoff_freq * timing::SAMPLE_PERIOD / 1000.0))),
   last_output(0) {}

  /**
   * \brief Actualiza la salida del filtro en base a la nueva entrada \p input
   * 
   * \param input La función de entrada en el tiempo actual $x(k)$
   * \return El valor filtrado $y(k)$ a partir de $x(k)$
   */
  double 
  operator()(double input)
  {
    last_output = alpha*input + (1 - alpha)*last_output;
    return last_output;
  }
};
