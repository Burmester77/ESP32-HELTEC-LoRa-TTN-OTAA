#ifndef SOILMOISTUREHANDLER_HPP
#define SOILMOISTUREHANDLER_HPP

#include <Arduino.h>

class SoilMoistureHandler 
{
public:
    void readData();
    int getRawMoistureValue();
    int getMappedMoistureValue();
};

extern SoilMoistureHandler soilMoistureHandler;

#endif