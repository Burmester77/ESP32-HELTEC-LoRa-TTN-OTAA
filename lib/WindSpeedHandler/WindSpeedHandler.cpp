#include "WindSpeedHandler.hpp"
#include <alog.h>

WindSpeedHandler windSpeedHandler;

#define anemoPin 47 // Wind speed sensor pin

// Variables for wind speed calculation
volatile int windCnt = 0;
float windspeed = 0.0;

void IRAM_ATTR WindSpeedHandler::windCounter() {
    windCnt++;
}

void WindSpeedHandler::setup() {
    pinMode(anemoPin, INPUT_PULLUP);
}

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

float WindSpeedHandler::getWindSpeed() {
    return windspeed;
}
