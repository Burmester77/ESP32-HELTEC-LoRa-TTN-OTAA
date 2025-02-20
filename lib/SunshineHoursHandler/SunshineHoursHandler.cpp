#include "SunshineHoursHandler.hpp"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <alog.h>


SunshineHoursHandler sunshineHoursHandler;

const int oneWireBus = 6; // DS18B20 on pin 6

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Variable to store the temperature
float DS18B20temperature = 0.0;

// Setup the DS18B20 sensor /////////////////////////////////
void SunshineHoursHandler::setup() {
    sensors.begin();
}

// Read the temperature from the DS18B20 sensor /////////////////////////////////
void SunshineHoursHandler::readData() {
    sensors.requestTemperatures();
    delay(100);
    DS18B20temperature = sensors.getTempCByIndex(0);
    ALOG_D("DS18B20 Temperatur: %.2f°C", DS18B20temperature);
    ALOG_D("Erkannte Sensoren: %d", sensors.getDeviceCount());
}

float SunshineHoursHandler::getDS18B20Temperature() {
    return DS18B20temperature;
}
