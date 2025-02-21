#include <WindSpeedHandler.hpp>
#include <alog.h>

// Global instance of WindSpeedHandler
WindSpeedHandler windSpeedHandler;

#define anemoPin 47 // Wind speed sensor pin

// Variables for wind speed calculation
volatile int windCnt = 0;
float windspeed = 0.0;

// Function to count the wind impulses ///////////////////////////////////////
void IRAM_ATTR WindSpeedHandler::windCounter() {
    windCnt++;
}

// Setup wind speed sensor ////////////////////////////////////////////////////////////
void WindSpeedHandler::setup() {
    pinMode(anemoPin, INPUT_PULLUP);
}

// Read wind speed ////////////////////////////////////////////////////////////
void WindSpeedHandler::readData() {
    windCnt = 0;
    attachInterrupt(digitalPinToInterrupt(anemoPin), windCounter, FALLING);
    uint32_t startTime = millis();
    while (millis() - startTime < 10000) { // 10 seconds measurement
        vTaskDelay(10);
    }
    detachInterrupt(anemoPin);
    windspeed = ((float)windCnt / 10.0 * 2.4) / 2; // Calculate wind speed in km/h (1 impulse = 2.4 km/h, 2 impulses per rotation, 10 seconds measurement)
    // ALOG_D("Impulse: %d, Windspeed: %.2f km/h", windCnt, windspeed);
}

// Get the wind speed ////////////////////////////////////////////////////////////
float WindSpeedHandler::getWindSpeed() {
    return windspeed;
}