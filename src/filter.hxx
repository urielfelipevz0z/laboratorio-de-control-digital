#ifndef FILTER_HXX
#define FILTER_HXX

namespace filter {
    class ExpLowPassFilter {
    private:
        double alpha;             /**< Coeficiente $\alpha$ de suavizado  */
        double last_output;       /**< Última muestra suavizada $y(k-1)$ */
    public:
        ExpLowPassFilter(double cutoff_freq);
        double operator()(double input);
    };
}

#endif // FILTER_HXX