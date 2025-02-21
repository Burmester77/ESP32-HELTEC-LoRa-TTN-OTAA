/*
 * Copyright 2025 Thorsten Ludewig (t.ludewig@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <Arduino.h>
#include <LoRaWANHandler.hpp>
#include <BatteryHandler.hpp>
#include <BME280Handler.hpp>
#include <LightIntensityHandler.hpp>
#include <RainHandler.hpp>
#include <SoilMoistureHandler.hpp>
#include <SunStatusHandler.hpp>
#include <WindDirectionHandler.hpp>
#include <WindSpeedHandler.hpp>
#include <rom/crc.h>
#include <alog.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <esp_sleep.h>


// I2C1-Pins (alternative pins for I2C1 because I2C0 is used by Display)
static const uint8_t SDA_I2C1 = 46;
static const uint8_t SCL_I2C1 = 45;

/*

Schedule downlink (FPort 1)

Payload Type: Bytes
  Byte 0: 0x5A (magic number) 
   - Why 0x5A? 0x5A is a nice binary number, '0101 1010', 
     and it's easy to recognize.

  Byte 1: Command 
  Byte 2-5: Value

Commands:
  0x01: Set sleep time
  0x02: Set send delay

Example:
0x5A 0x01 0x00 0x01 0xD4 0xC0 -> Set sleep time to 120000ms (2 minutes)

The values are in big-endian format and represent a 32-bit unsigned integer.
They are permanently stored in the device's non-volatile memory.

*/

// Function to wait non-blocking for a certain time //////////////////////////////////////////////
void wait(uint64_t delay_ms){
  uint64_t start = esp_timer_get_time();
  while (esp_timer_get_time() - start < delay_ms * 1000ULL){
    vTaskDelay(10);
  }
}

/**
 * @brief Prepares the transmission frame for LoRaWAN.
 *
 * This function sets up the application data to be sent over LoRaWAN.
 * If a send delay is configured, it applies the delay before preparing the frame.
 *
 * @param port The port number on which to send the data.
 */
// uint8_t appData[255]
void prepareTxFrame(uint8_t port)
{
  // ALOG_D("Wake up reason: %d", esp_sleep_get_wakeup_cause());

  // Read defined send delay - sensor-warm-up time
  if ( loRaWANHandler.getSendDelay() > 0 )
  {
    ALOG_D("Send delay: %dms", loRaWANHandler.getSendDelay());
    wait(loRaWANHandler.getSendDelay());
  }

  // Read BATTERY VOLTAGE //////////////////////////////////////////////
  float voltage = batteryHandler.getBatteryVoltage();
  uint16_t voltageInt = (uint16_t)(voltage * 1000); // Calculate battery voltage in mV
  ALOG_D("Battery voltage: %.2f", voltage);

  // Read BME280 sensor data //////////////////////////////////////////////
  bme280Handler.readData();
  float temp = bme280Handler.getTemperature();
  float hum = bme280Handler.getHumidity();
  float pres = bme280Handler.getPressure();
  ALOG_D("Temperature: %.2f °C", temp);
  ALOG_D("Humidity: %.2f %%", hum);
  ALOG_D("Pressure: %.2f hPa", pres);
  int16_t tempInt = (int16_t)((temp * 100) + 5000); // Calculate temperature in 0.01°C steps and add offset because of negative values
  uint16_t pressureInt = (uint16_t)(pres * 10);   // Calculate air pressure in 0.1 hPa steps
  uint8_t humInt = (uint8_t)(hum * 2);    // Calculate humidity in 0.5% steps

  // Read LIGHT INTENSITY data //////////////////////////////////////////////
  lightIntensityHandler.readData();
  uint8_t lightInt = lightIntensityHandler.getNormalizedLightValue();
  ALOG_D("Light intensity - Raw ADC Value: %d", lightIntensityHandler.getRawLightValue());

  // Read RAIN GAUGE data //////////////////////////////////////////////
  float rainAmount = rainHandler.getRainAmount();
  ALOG_D("Rain amount: %.2f mm", rainAmount);
  uint16_t rainAmountInt = (uint16_t)(rainAmount * 10); // Calculate rain amount in 0.1 mm steps  

  // Read SOIL MOISTURE data //////////////////////////////////////////////
  soilMoistureHandler.readData();
  int soilMoisture = soilMoistureHandler.getMappedMoistureValue();
  ALOG_D("Soil moisture: %d", soilMoisture);
  uint8_t soilMoistureInt = (uint8_t)(soilMoisture * 2);

  // Read SUN STATUS data ////////////////////////////////////////////
  sunStatusHandler.readData();
  float sunShining = sunStatusHandler.isSunShining();
  uint8_t sunStatus = sunShining ? 1 : 0; // 1 = sun, 0 = no sun 
  ALOG_D("Sunshine status: %s", sunShining ? "Sun" : "No sun");

  // Read wind direction data //////////////////////////////////////////////
  windDirectionHandler.readData();
  float windDirection = windDirectionHandler.getWindDirection();
  String windDirectionLabel = windDirectionHandler.getWindDirectionLabel(windDirection);
  ALOG_D("Wind Direction: %.1f°, %s", windDirection, windDirectionLabel.c_str());
  uint16_t windDirectionInt = (uint16_t)(windDirection / 22.5); // Calculate wind direction in 22.5° steps

  // Read wind speed data //////////////////////////////////////////////
  ALOG_D("Measuring wind speed (10 seconds)...");
  windSpeedHandler.readData();
  float windspeed = windSpeedHandler.getWindSpeed();
  ALOG_D("Wind Speed: %.2f km/h", windspeed);
  uint16_t windInt = (uint16_t)(windspeed * 10); // Calculate wind speed in 0.1 km/h steps
 
  
  // Build LoRaWAN data frame //////////////////////////////////////////////
  appDataSize = 19; // Size set to 18 bytes
  appData[0] = 0x5A;
  appData[1] = 0x01;
  appData[2] = (tempInt >> 8) & 0xFF;
  appData[3] = tempInt & 0xFF;
  appData[4] = humInt;
  appData[5] = (pressureInt >> 8) & 0xFF;
  appData[6] = pressureInt & 0xFF;
  appData[7] = (windInt >> 8) & 0xFF;
  appData[8] = windInt & 0xFF;
  appData[9] = (rainAmountInt >> 8) & 0xFF;
  appData[10] = rainAmountInt & 0xFF;
  appData[11] = (windDirectionInt >> 8) & 0xFF;
  appData[12] = windDirectionInt & 0xFF;
  appData[13] = (voltageInt >> 8) & 0xFF;
  appData[14] = voltageInt & 0xFF;
  appData[15] = lightInt;
  appData[16] = soilMoistureInt;
  appData[17] = sunStatus;

  // CRC8 for the first 18 bytes
  appData[18] = crc8_le(0, appData, 8);
}

/**
 * @brief Initializes the LoRaWAN handler.
 *
 * This function sets up the LoRaWAN handler by invoking the setup method
 * of the loRaWANHander object during the initialization phase.
 */
void setup()
{
  // Initialize Wire1 for I2C1 (alternative pins) and configure sensor pins
  Wire1.begin(SDA_I2C1, SCL_I2C1);

  batteryHandler.setup();
  bme280Handler.setup();
  lightIntensityHandler.setup();
  loRaWANHandler.setup();
  rainHandler.setup();
  sunStatusHandler.setup();
  windDirectionHandler.setup();
  windSpeedHandler.setup();
}

/**
 * @brief Continuously handles LoRaWAN events and maintains the connection.
 *
 * This function is repeatedly called in the main loop and delegates
 * processing to the LoRaWAN handler's loop method. It ensures that
 * LoRaWAN events are processed and the connection remains active.
 */
void loop()
{
  loRaWANHandler.loop();
}
