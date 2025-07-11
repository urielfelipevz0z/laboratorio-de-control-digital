#ifndef GLOBALS_HXX
#define GLOBALS_HXX

using millis_t = unsigned long;
namespace pin {
  constexpr int MISO = 5;
  constexpr int MOSI = 4;
}

namespace timing {
  constexpr millis_t SAMPLE_PERIOD = 10;
  constexpr double NYQUIST_FREQ 
    = 1 / (2 * (SAMPLE_PERIOD / 1000.0)); /**< Frecuencia de Nyquist para la 
                                               toma de muestras sin pérdida
                                               de información */
}

constexpr int16_t MAX_OUT_VALUE = 255;
constexpr int16_t MAX_IN_VALUE = 4095;
constexpr double MAX_VOLTAGE = 3.3;

#endif // GLOBALS_HXX