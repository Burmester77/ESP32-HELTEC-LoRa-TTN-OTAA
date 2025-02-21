#include "SoilMoistureHandler.hpp"
#include <alog.h>

#define soilPin 3 // SoilWatch10 sensor pin

// Global instance of SoilMoistureHandler
SoilMoistureHandler soilMoistureHandler;

const int minADC = 6;         // Dry value (air)
const int maxADC = 3437;       // Wet value (water)

// Variables for sensor data
int rawMoistureValue = 0;   // Raw ADC value
int mappedMoistureValue = 0; // Mapped value in percent

// Read soil moisture data ////////////////////////////////////////////////////////////
void SoilMoistureHandler::readData() {
    rawMoistureValue = analogRead(soilPin);
    mappedMoistureValue = map(rawMoistureValue, minADC, maxADC, 0, 100);
    // ALOG_D("Soil Moisture: ADC = %d, Mapped Value = %d%%", rawMoistureValue, mappedMoistureValue);
}

// Get raw moisture value ////////////////////////////////////////////////////////////
int SoilMoistureHandler::getRawMoistureValue() {
    return rawMoistureValue;
}

// Get mapped moisture value ////////////////////////////////////////////////////////////
int SoilMoistureHandler::getMappedMoistureValue() {
    return mappedMoistureValue;
}
