#include <SunStatusHandler.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <alog.h>

// Global instance of SunshineHoursHandler
SunStatusHandler sunStatusHandler;

const int oneWireBus = 6; // DS18B20 on pin 6

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Variable to store the temperature
float DS18B20temperature = 0.0;
const float sunshineThreshold = 9.0; // Temperature difference threshold for sunshine detection

// Setup the DS18B20 sensor //////////////////////////////////////////
void SunStatusHandler::setup() {
    sensors.begin();
}

// Read the temperature from the DS18B20 sensor /////////////////////////////////
void SunStatusHandler::readData() {
    sensors.requestTemperatures();
    delay(100);
    DS18B20temperature = sensors.getTempCByIndex(0);
    //ALOG_D("DS18B20 Temperatur: %.2f°C", DS18B20temperature);
}

// Get the temperature from the DS18B20 sensor /////////////////////////////////
float SunStatusHandler::getDS18B20Temperature() {
    return DS18B20temperature;
}

// Check if the sun is shining //////////////////////////////////////////
bool SunStatusHandler::isSunShining() {
    float bme280Temperature = bme280Handler.getTemperature();
    // Calculate the temperature difference between the BME280 and DS18B20 sensor
    float temperatureDifference = bme280Temperature - DS18B20temperature;
    // ALOG_D("Temperature difference: %.2f", temperatureDifference);
    // If the temperature difference is greater than the threshold, the sun is shining
    if (temperatureDifference > sunshineThreshold) {
        return true;
    }
    return false;
}