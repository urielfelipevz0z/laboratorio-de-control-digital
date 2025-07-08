#ifndef DIRTY_DERIVATIVE_HXX
#define DIRTY_DERIVATIVE_HXX

#include "globals.hxx"
#include "filter.hxx"

/**
 * \brief Esta clase se encarga de realizar una derivada sucia en tiempo real
 * 
 * Como tal se recibe una nueva muestra de la señal y se calcula la derivada
 * después de aplicar un filtrado de primer orden a la señal. Implementamos la
 * siguiente ecuación de diferencias:
 *   $$ y(k) = \frac{x(k) - x(k-1)}{T} $$
 * Donde $x(k)$ es la entrada ruidosa y $T$ es el periodo de muestreo.
 */
class DirtyDerivative {
private:
  filter::ExpLowPassFilter filter; /**< Filtro para suavizar y evitar picos de ruido */
  double last_input;
public:
  DirtyDerivative(double cutoff_freq = timing::NYQUIST_FREQ);
  double operator()(double input);

};

#endif // DIRTY_DERIVATIVE_HXX