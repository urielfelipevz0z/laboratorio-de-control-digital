#ifndef PID_CONTROLLER_HXX
#define PID_CONTROLLER_HXX
#include "DirtyDerivative.hxx"

/**
 * \brief Controlador PID digital listo para actuar en sistemas de tiempo real
 * 
 * Se trata de un controlador configurable, con posibilidad de sintonizarse con
 * diferentes tipos de plantas de primer y segundo orden a través de varias reglas
 * empíricas y de optimización.
 * 
 * TODO: Generalizar este controlador a ON-OFF y P en la medida de lo posible.
 * 
 */
class PidController {
private:
    double k_p;  ///< Ganancia proporcional del PID
    double k_i;  ///< Ganancia integral del PID  
    double k_d;  ///< Ganancia derivativa del PID
    
    double T_i;  ///< Tiempo integral (Ti = ki/kp) - usado para cálculo de antiwindup
    double T_d;  ///< Tiempo derivativo (Td = kd/kp) - usado para cálculo de antiwindup
    double k_r;  ///< Ganancia de rastreo/antiwindup (kr = 1/√(Ti*Td))

    bool antiwindup = true;  ///< Habilita/deshabilita el mecanismo antiwindup

    DirtyDerivative derivative;

    static std::tuple<double, double, double> takahashi(double L, double R, double k);
    static std::tuple<double, double, double> optimize_genetic(double k_p, double k_i, double k_d);

public:
    PidController(double k_p = 3.0, double k_i = 1.0, double k_d = 1.0);
    void tune();
    int16_t operator()(int16_t error);
};

#endif // PID_CONTROLLER_HXX