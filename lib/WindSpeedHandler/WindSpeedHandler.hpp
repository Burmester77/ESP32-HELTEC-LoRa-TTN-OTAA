#ifndef WINDSPEEDHANDLER_HPP
#define WINDSPEEDHANDLER_HPP

#include <Arduino.h>

class WindSpeedHandler {
public:
    void setup();
    void readData();
    float getWindSpeed();

private:
    static void IRAM_ATTR windCounter();
};

extern WindSpeedHandler windSpeedHandler;

#endif
