#include <Arduino.h>

#include "DirtyDerivative.hxx"

using namespace filter;

/**
 * \brief Crea un derivador sucio listo para recibir valores a partir de la
 *        frecuencia de corte \p cutoff_freq específicada
 * 
 * \note En caso de no especificarse \p cutoff_freq se tomará la máxima
 *       frecuencia reconstruible del sistema $f_N$
 * 
 * \param cutoff_freq Frecuencia de corte del filtro pasa bajas (en Hz)
 */
DirtyDerivative::DirtyDerivative(double cutoff_freq):
    filter(cutoff_freq),
    last_input(0) {}

/**
 * \brief Obtiene la derivada sucia de la señal dada la entrada \p input
 * 
 * \param input Entrada ruidosa de la señal a derivar $x(k)$
 * \return La derivada sucia $y(k)$ a partir de la entrada $x(k)$
 */
double 
DirtyDerivative::operator()(double input)
{
    input = filter(input);
    const double derivative = (input - last_input) / (timing::SAMPLE_PERIOD / 1000.0) ;
    last_input = input;
    return derivative;
  }
};