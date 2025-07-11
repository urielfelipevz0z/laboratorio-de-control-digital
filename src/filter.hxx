#ifndef FILTER_HXX
#define FILTER_HXX

#include "globals.hxx"

namespace filter {
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
        ExpLowPassFilter(double cutoff_freq  = timing::NYQUIST_FREQ);
        int16_t operator()(double input);
    };
}

#endif // FILTER_HXX