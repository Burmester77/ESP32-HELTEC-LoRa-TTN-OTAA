#ifndef SUNSTATUSHANDLER_HPP
#define SUNSTATUSHANDLER_HPP

#include <Arduino.h>
#include <BME280Handler.hpp>

class SunStatusHandler {
public:
    void setup();
    void readData();
    float getDS18B20Temperature();
    bool isSunShining();
};

extern SunStatusHandler sunStatusHandler;

#endif
