#include <LightIntensityHandler.hpp>
#include <alog.h>

#define lightPin 5 // Light sensor pin

// Global instance of LightIntensityHandler
LightIntensityHandler lightIntensityHandler;

// Variables to store light intensity values
int rawLightValue;
uint8_t normalizedLightValue;

// Setup light sensor ////////////////////////////////////////////////////////////
void LightIntensityHandler::setup()
{
    pinMode(lightPin, INPUT);
    analogSetPinAttenuation(lightPin, ADC_11db);
}

// Read light intensity data ////////////////////////////////////////////////////////////
void LightIntensityHandler::readData()
{
    rawLightValue = analogRead(lightPin);
    normalizedLightValue = (uint8_t)(rawLightValue * 255 / 4095);
    // ALOG_D("Light Sensor - Raw Value: %d, Normalized Value: %d", rawLightValue, normalizedLightValue);
}

// Get raw light value ////////////////////////////////////////////////////////////
int LightIntensityHandler::getRawLightValue()
{
    return rawLightValue;
}

// Get normalized light value ////////////////////////////////////////////////////////////
uint8_t LightIntensityHandler::getNormalizedLightValue()
{
    return normalizedLightValue;
}