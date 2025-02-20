#ifndef SUNSHINEHOURSHANDLER_HPP
#define SUNSHINEHOURSHANDLER_HPP

#include <Arduino.h>

class SunshineHoursHandler {
public:
    void setup();
    void readData();
    float getDS18B20Temperature();
};

extern SunshineHoursHandler sunshineHoursHandler;

#endif
