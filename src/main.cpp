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
//#include <BatteryHandler.hpp>
#include <rom/crc.h>
#include <alog.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <esp_sleep.h>

// Sensor-Pins
#define anemoPin 47
#define rainPin 7
#define windVanePin 4

// BME280 Sensor object initialisation
Adafruit_BME280 bme;

// Globale variables for sensor data
float temperature = 0.0;        // Temperatur in °C
float humidity = 0.0;           // Humidity in %
float pressure = 0.0;           // Airpressure in hPa 
volatile int windCnt = 0;       // Counter for wind impulses
float windspeed;                // Windspeed km/h
RTC_DATA_ATTR volatile int rainCnt = 0;       // Counter for rain impulses
RTC_DATA_ATTR unsigned long lastRainReset = 0; // Time of last rain reset
RTC_DATA_ATTR float hourlyRainAmount = 0.0;          // hourly rain amount in mm 
RTC_DATA_ATTR float rainAmount = 0.0;          // measured rain amount in mm
unsigned long lastRainTipTime = 0; // Time of last rain tip
const unsigned long debounceInterval = 100; // Debounce interval for rain gauge
int adcValue; // ADC value for wind direction
float windDirection; // Wind direction in degrees

// I2C1-Pins (alternative pins for I2C1 because I2C0 is used by Display)
static const uint8_t SDA_I2C1 = 45;
static const uint8_t SCL_I2C1 = 46;

// Wind direction
// Enum to define the indexes for each wind direction
enum WindVaneAngles
{
    ANGLE_0_0 = 0,
    ANGLE_22_5,
    ANGLE_45_0,
    ANGLE_67_5,
    ANGLE_90_0,
    ANGLE_112_5,
    ANGLE_135_0,
    ANGLE_157_5,
    ANGLE_180_0,
    ANGLE_202_5,
    ANGLE_225_0,
    ANGLE_247_5,
    ANGLE_270_0,
    ANGLE_292_5,
    ANGLE_315_0,
    ANGLE_337_5,
    NUM_ANGLES
};

// Angle per index of wind vane (360 / 16 = 22.5)
#define WIND_VANE_DEGREES_PER_INDEX (360.0 / NUM_ANGLES)

    // ADC values for each wind direction (V(Wind Vane) / 3.3V * 4095)
    #define ADC_ANGLE_0_0 2960
    #define ADC_ANGLE_22_5 1482
    #define ADC_ANGLE_45_0 1693
    #define ADC_ANGLE_67_5 281
    #define ADC_ANGLE_90_0 310
    #define ADC_ANGLE_112_5 211
    #define ADC_ANGLE_135_0 652
    #define ADC_ANGLE_157_5 439
    #define ADC_ANGLE_180_0 1036
    #define ADC_ANGLE_202_5 870
    #define ADC_ANGLE_225_0 2339
    #define ADC_ANGLE_247_5 2221
    #define ADC_ANGLE_270_0 3815
    #define ADC_ANGLE_292_5 3154
    #define ADC_ANGLE_315_0 3467
    #define ADC_ANGLE_337_5 2618

    #define ADC_RESOLUTION 12

String getWindDirectionLabel(float windDirection) {
    if (windDirection == 0.0) return "N";
    else if (windDirection == 22.5) return "NNE"; 
    else if (windDirection == 45.0) return "NE";
    else if (windDirection == 67.5) return "ENE";
    else if (windDirection == 90.0) return "E";
    else if (windDirection == 112.5) return "ESE";
    else if (windDirection == 135.0) return "SE";
    else if (windDirection == 157.5) return "SSE";
    else if (windDirection == 180.0) return "S";
    else if (windDirection == 202.5) return "SSW";
    else if (windDirection == 225.0) return "SW";
    else if (windDirection == 247.5) return "WSW";
    else if (windDirection == 270.0) return "W";
    else if (windDirection == 292.5) return "WNW";
    else if (windDirection == 315.0) return "NW";
    else if (windDirection == 337.5) return "NNW";
    else return "Unknown";
}
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

// Anemometer Interrupt-Routine
void windCounter(){
  windCnt++;
}

// Function to measure the wind speed
void measureWind() {
  attachInterrupt(digitalPinToInterrupt(anemoPin), windCounter , FALLING);
  delay(3000); // 3 Sekunden messen
  detachInterrupt(anemoPin);
  windspeed = ((float)windCnt / (float)3 * 2.4) / 2;
  ALOG_D("Impulse: %d, Windspeed: %.2f", windCnt, windspeed);

    windCnt = 0;
  }

// Function to count the rain tips 
void IRAM_ATTR rainCounter(){
    unsigned long currentTime = millis();
  if (currentTime - lastRainTipTime > debounceInterval) {
    rainCnt++;
    lastRainTipTime = currentTime;
  }
}

// Function to measure the rain amount
void measureRain() {

  rainAmount = (float)rainCnt * 0.2794;
  hourlyRainAmount += rainAmount;
  ALOG_D("Regenimpulse: %d, Regenmenge: %.2f mm, stündl.Regen: %.2f mm", rainCnt, rainAmount, hourlyRainAmount);
  rainCnt = 0;


  unsigned long currentMillis = millis();
  if (currentMillis - lastRainReset > 3600000) {
    hourlyRainAmount = 0.0;
    lastRainReset = currentMillis;
  }

}


float measureWindDirection(){

    // Read the wind vane ADC value
  int adcValue = analogRead(windVanePin);

  // ADC values for each wind direction (V(Wind Vane) / 3.3V * 4095)
    int adcValues[NUM_ANGLES] = {
        ADC_ANGLE_0_0, ADC_ANGLE_22_5, ADC_ANGLE_45_0, ADC_ANGLE_67_5,
        ADC_ANGLE_90_0, ADC_ANGLE_112_5, ADC_ANGLE_135_0, ADC_ANGLE_157_5,
        ADC_ANGLE_180_0, ADC_ANGLE_202_5, ADC_ANGLE_225_0, ADC_ANGLE_247_5,
        ADC_ANGLE_270_0, ADC_ANGLE_292_5, ADC_ANGLE_315_0, ADC_ANGLE_337_5
    };
  // Wind angles for each wind direction
    float windAngles[NUM_ANGLES] = {
        0.0, 22.5, 45.0, 67.5, 90.0, 112.5, 135.0, 157.5,
        180.0, 202.5, 225.0, 247.5, 270.0, 292.5, 315.0, 337.5
    };

    // Find the closest ADC value to the measured value
    int closestIndex = 0;
    int minDifference = abs(adcValue - adcValues[0]);

    // Iterate through all ADC values and find the closest match
    for (int i = 1; i < NUM_ANGLES; i++) {
        int difference = abs(adcValue - adcValues[i]);
        // If the difference is smaller than the current minimum difference, update the minimum difference and the closest index
        if (difference < minDifference) {
            minDifference = difference;
            closestIndex = i;
        }
    }

    float windDirection = windAngles[closestIndex];
    ALOG_D("ADC Value: %d, Closest Match: %d, Wind Direction: %.1f°", adcValue, adcValues[closestIndex], windDirection);

    return windDirection;
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
  // e.g. warmup delay for the sensor
  if ( loRaWANHandler.getSendDelay() > 0 )
  {
    ALOG_D("Send delay: %dms", loRaWANHandler.getSendDelay());
    delay(loRaWANHandler.getSendDelay());
  }

/*   float voltage = batteryHandler.getBatteryVoltage();
  ALOG_D("Battery voltage: %.2fV", voltage);
  voltage -= 2;
  voltage *= 100;
  uint8_t voltageInt = (uint8_t)voltage;
  ALOG_D("Voltage Data: %d", voltageInt); */

  // Read BME280 sensor data
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  ALOG_D("Temperature: %.2f °C", temperature);
  ALOG_D("Humidity: %.2f %%", humidity);
  ALOG_D("Pressure: %.2f hPa", pressure);

  // Calculate temperature in 0.01°C steps and add offset because of negative values
  int16_t tempInt = (int16_t)((temperature * 100) + 5000);
  
  // Calculate air pressure in 0.1 hPa steps
  uint16_t pressureInt = (uint16_t)(pressure * 10);

  // Calculate humidity in 0.5% steps
  uint8_t humInt = (uint8_t)(humidity * 2);
  ALOG_D("Humidity value (scaled): %.2f, Humidity byte: %d", humidity, humInt);

  measureWind();
  uint16_t windInt = (uint16_t)(windspeed * 10);

  measureRain();
  uint16_t rainInt = (uint16_t)(rainAmount * 10);
  uint16_t hourlyRainInt = (uint16_t)(hourlyRainAmount * 10);

  float windDirection = measureWindDirection();
  String windDirectionLabel = getWindDirectionLabel(windDirection);
  ALOG_D("Wind Direction: %.1f°, %s", windDirection, windDirectionLabel.c_str());

  uint16_t windDirectionInt = (uint16_t)(windDirection / 22.5);
  ALOG_D("Wind Direction Int: %d", windDirectionInt);
 

  // LoRaWAN-Frame aufbauen
  appDataSize = 16; // Größe wird auf 8 Bytes gesetzt
  appData[0] = 0x5A;                   // Preamble
  appData[1] = 0x01;                   // Sensorstatus
  //appData[2] = voltageInt;             // Batteriespannung
  appData[2] = (tempInt >> 8) & 0xFF;  // Temperatur MSB
  appData[3] = tempInt & 0xFF;         // Temperatur LSB
  appData[4] = humInt;                 // Luftfeuchtigkeit
  appData[5] = (pressureInt >> 8) & 0xFF; // Luftdruck MSB
  appData[6] = pressureInt & 0xFF;     // Luftdruck LSB
  appData[7] = (windInt >> 8) & 0xFF;  // Windgeschwindigkeit MSB
  appData[8] = windInt & 0xFF;         // Windgeschwindigkeit LSB
  appData[9] = (rainInt >> 8) & 0xFF; // Regenmenge MSB
  appData[10] = rainInt & 0xFF;        // Regenmenge LSB
  appData[11] = (hourlyRainInt >> 8) & 0xFF; // stündliche Regenmenge MSB
  appData[12] = hourlyRainInt & 0xFF;        // stündliche Regenmenge LSB
  appData[13] = (windDirectionInt >> 8) & 0xFF; // Windrichtung MSB
  appData[14] = windDirectionInt & 0xFF;        // Windrichtung LSB


  // CRC8 über die ersten 7 Bytes berechnen und ans Ende schreiben
  appData[15] = crc8_le(0, appData, 8);

  /*
  // Ausgabe der Frame-Daten
  ALOG_D("LoRaWAN Frame Data:");
  for (uint8_t i = 0; i < appDataSize + 1; i++) // +1 für CRC8
  {
    ALOG_D("Byte %d: 0x%02X", i, appData[i]);
  }
  */
}

/**
 * @brief Initializes the LoRaWAN handler.
 *
 * This function sets up the LoRaWAN handler by invoking the setup method
 * of the loRaWANHander object during the initialization phase.
 */
void setup()
{
  // I2C1 für den BME280 initialisieren
  Wire1.begin(SDA_I2C1, SCL_I2C1);
  pinMode(anemoPin, INPUT_PULLUP);
  pinMode(rainPin, INPUT_PULLUP);
  pinMode(windVanePin, INPUT);
  

  attachInterrupt(digitalPinToInterrupt(rainPin), rainCounter, FALLING);

  // Configure external wakeup on rain gauge pin (GPIO_NUM_7)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    if (digitalRead(rainPin) == LOW) {  // Regenmesser ist noch LOW?
      rainCnt++;  // Ersten Impuls nach dem Wake-Up mitzählen
    }
  }

  // BME280 initialisieren
  if (!bme.begin(0x76, &Wire1)) {
    ALOG_E("BME280 nicht gefunden. Starte ohne Sensorwerte.");
  } else {
    ALOG_D("BME280 erfolgreich initialisiert.");
  }

  //batteryHandler.setup();
  loRaWANHandler.setup();
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
