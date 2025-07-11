#include <Arduino.h>

#include "OnOffController.hxx"
#include "globals.hxx"

OnOffController::OnOffController(double vh)
: vh(vh) 
{
  if (vh < 0) {
    this->vh = 0.0;
  }
}

int16_t
OnOffController::operator()(int16_t input) 
{
    static int16_t last_output = 0; // Variable estática para mantener estado de histéresis
    
    if (input >= (2.5 + vh)) {
        last_output = MAX_OUT_VALUE; // Activa el controlador
        return last_output;
    } else if (input <= (2.5 - vh)) {
        last_output = 0; // Desactiva el controlador
        return last_output;
    } else {
        // Estado de histéresis - mantener último valor
        return last_output;
    }
}