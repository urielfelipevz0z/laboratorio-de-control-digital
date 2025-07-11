#ifndef ON_OFF_CONTROLLER_HXX
#define ON_OFF_CONTROLLER_HXX
#include <Arduino.h>
#include "globals.hxx"


/**
 * \brief Esta clase se encarga de realizar un control On/Off en tiempo real
 * 
 * Como tal se recibe una nueva muestra de la señal y se calcula la salida
 * del controlador en función de un umbral.
 */
class OnOffController {
private:
  double vh; /**< Umbral de activación del controlador */
public:
  OnOffController(double vh = 0.0);
  int16_t operator()(int16_t input);
};

#endif // ON_OFF_CONTROLLER_HXX