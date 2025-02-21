#ifndef LIGHTINTENSITYHANDLER_HPP
#define LIGHTINTENSITYHANDLER_HPP

#include <Arduino.h>

class LightIntensityHandler
{
  public:
    void setup();
    void readData();
    int getRawLightValue();
    uint8_t getNormalizedLightValue();
};

extern LightIntensityHandler lightIntensityHandler;

#endif
