#include <microsmooth.h>


#include "SmoothPin.h"

SmoothPin::SmoothPin() {}

void SmoothPin::init(int _pin, bool pullup){
    pin = _pin;
    if (pullup){
      pinMode(_pin, INPUT_PULLUP);
    } else {
      pinMode(_pin, INPUT);
    }
    ptr = ms_init(SMA);
}

int SmoothPin::read(){
  int val = analogRead(pin);
  value = sma_filter(val, ptr);
  return value;
}
