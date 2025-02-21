#include <BME280Handler.hpp>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <alog.h>

// Global instance of BME280Handler
BME280Handler bme280Handler;

// Initialize BME280 sensor
Adafruit_BME280 bme;

// Variables for sensor data
float temperature = 0.0;    // in °C
float humidity = 0.0;       // in %
float pressure = 0.0;       // in hPa

// Setup BME280 sensor ////////////////////////////////////////////////////////////
void BME280Handler::setup() {
    if (!bme.begin(0x76, &Wire1)) { // BME280 standard address -> 0x76
        ALOG_E("BME280 not found.");
    } else {
        ALOG_D("BME280 successfully initialized.");
    }
}

// Read BME280 sensor data ////////////////////////////////////////////////////////////
void BME280Handler::readData() {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F; // convert to hPa
    //ALOG_D("BME280 - Temp: %.2f°C, Hum: %.2f%%, Press: %.2f hPa", temperature, humidity, pressure);
}

// Get temperature data ////////////////////////////////////////////////////////////
float BME280Handler::getTemperature() {
    return temperature;
}

// Get humidity data ////////////////////////////////////////////////////////////
float BME280Handler::getHumidity() {
    return humidity;
}

// Get pressure  data ////////////////////////////////////////////////////////////
float BME280Handler::getPressure() {
    return pressure;
}
