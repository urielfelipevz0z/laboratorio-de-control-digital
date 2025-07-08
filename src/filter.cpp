#include <Arduino.h>
#include <cmath>
#include <tuple>
#include <utility>

#include "filter.hxx"
#include "globals.hxx"

namespace filter {
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
ExpLowPassFilter::ExpLowPassFilter(double cutoff_freq):
  alpha(exp(- (2.0 * M_PI * cutoff_freq * timing::SAMPLE_PERIOD / 1000.0))),
  last_output(0) {}

/**
 * \brief Actualiza la salida del filtro en base a la nueva entrada \p input
 * 
 * \param input La función de entrada en el tiempo actual $x(k)$
 * \return El valor filtrado $y(k)$ a partir de $x(k)$
 */
double ExpLowPassFilter::operator()(double input)
{
  last_output = alpha*input + (1 - alpha)*last_output;
  return last_output;
}
} // namespace filter