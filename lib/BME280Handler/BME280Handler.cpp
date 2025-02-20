#include "BME280Handler.hpp"
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <alog.h>

BME280Handler bme280Handler;

// Initialize BME280 sensor
Adafruit_BME280 bme;

// Variables for sensor data
float temperature = 0.0;    // in °C
float humidity = 0.0;       // in %
float pressure = 0.0;       // in hPa

void BME280Handler::setup() {
    if (!bme.begin(0x76, &Wire1)) { // BME280 standard address is 0x76
        ALOG_E("BME280 nicht gefunden.");
    } else {
        ALOG_D("BME280 erfolgreich initialisiert.");
    }
}

void BME280Handler::readData() {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F; // convert to hPa
    ALOG_D("BME280 - Temp: %.2f°C, Hum: %.2f%%, Press: %.2f hPa", temperature, humidity, pressure);
}

float BME280Handler::getTemperature() {
    return temperature;
}

float BME280Handler::getHumidity() {
    return humidity;
}

float BME280Handler::getPressure() {
    return pressure;
}
