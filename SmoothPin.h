#ifndef Smooth_h
#define Smooth_h
#include "Arduino.h"
class SmoothPin
{
public:
  SmoothPin();
  void init(int _pin, bool pullup);
  int read();
  int pin, rangelow, rangehigh, value;
  uint16_t *ptr;
};

#endif
