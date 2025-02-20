#ifndef BME280HANDLER_HPP
#define BME280HANDLER_HPP

#include <Arduino.h>

class BME280Handler {
public:
    void setup();
    void readData();
    float getTemperature();
    float getHumidity();
    float getPressure();
};

extern BME280Handler bme280Handler;

#endif
