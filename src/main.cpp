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
#include <rom/crc.h>
#include <alog.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <esp_sleep.h>

// Pin für den Anemometer (Windmesser) definieren
#define anemoPin 47
#define rainPin 7

// BME280 Sensor-Objekt erstellen
Adafruit_BME280 bme;

// Globale Variablen für Sensorwerte
float temperature = 0.0;        // Temperatur in °C
float humidity = 0.0;           // Luftfeuchtigkeit in %
float pressure = 0.0;           // Luftdruck in hPa 
volatile int windCnt = 0;       // Anzahl der Impulse des Anemometers
float windspeed;                // Windgeschwindigkeit in km/h
RTC_DATA_ATTR volatile int rainCnt = 0;       // Anzahl der Impulse des Regenmesser
RTC_DATA_ATTR unsigned long lastRainReset = 0; // Zeitpunkt des letzten Resets des Regenmessers
RTC_DATA_ATTR float hourlyRainAmount = 0.0;          // Regenmenge in mm
RTC_DATA_ATTR float rainAmount = 0.0;          // Regenmenge in mm
unsigned long lastRainTipTime = 0; // Zeitpunkt des letzten Regenimpulses
const unsigned long debounceInterval = 100; // Entprellzeit für den Regenmesser

// I2C1-Pins (alternative Pins für den BME280, da I2C0 für das Display verwendet wird)
static const uint8_t SDA_I2C1 = 45;
static const uint8_t SCL_I2C1 = 46;


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

// Funktion für die Interrupt-Routine des Anemometers
void windCounter(){
  windCnt++;
}

// Funktion zur Messung der Windgeschwindigkeit
void measureWind() {
  attachInterrupt(digitalPinToInterrupt(anemoPin), windCounter , FALLING);
  delay(3000); // 3 Sekunden messen
  detachInterrupt(anemoPin);
  windspeed = ((float)windCnt / (float)3 * 2.4) / 2;
  ALOG_D("Impulse: %d, Windspeed: %.2f", windCnt, windspeed);

    windCnt = 0;
  }

// Funktion für die Interrupt-Routine des Regenmessers
void IRAM_ATTR rainCounter(){
    unsigned long currentTime = millis();
  if (currentTime - lastRainTipTime > debounceInterval) {
    rainCnt++;
    lastRainTipTime = currentTime;
  }
}

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

  float voltage = batteryHandler.getBatteryVoltage();
  ALOG_D("Battery voltage: %.2fV", voltage);
  voltage -= 2;
  voltage *= 100;
  uint8_t voltageInt = (uint8_t)voltage;
  ALOG_D("Voltage Data: %d", voltageInt);

  // BME280 Sensorwerte auslesen
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  ALOG_D("Temperature: %.2f °C", temperature);
  ALOG_D("Humidity: %.2f %%", humidity);
  ALOG_D("Pressure: %.2f hPa", pressure);

  // Temperatur in Integer umrechnen (offset +5000 für Vorzeichenkorrektur)
  int16_t tempInt = (int16_t)((temperature * 100) + 5000);
  
  // Luftdruck in Integer umrechnen (in 0.1 hPa)
  uint16_t pressureInt = (uint16_t)(pressure * 10);

  // Luftfeuchtigkeit in 0.5%-Schritten kodieren
  uint8_t humInt = (uint8_t)(humidity * 2);
  ALOG_D("Humidity value (scaled): %.2f, Humidity byte: %d", humidity, humInt);

  // Windgeschwindigkeit messen
  measureWind();
  uint16_t windInt = (uint16_t)(windspeed * 10);

  measureRain();
  // Regenmenge in Integer umrechnen (in 0.1 mm)
  uint16_t rainInt = (uint16_t)(rainAmount * 10);
  uint16_t hourlyRainInt = (uint16_t)(hourlyRainAmount * 10);


  // LoRaWAN-Frame aufbauen
  appDataSize = 15; // Größe wird auf 8 Bytes gesetzt
  appData[0] = 0x5A;                   // Preamble
  appData[1] = 0x01;                   // Sensorstatus
  appData[2] = voltageInt;             // Batteriespannung
  appData[3] = (tempInt >> 8) & 0xFF;  // Temperatur MSB
  appData[4] = tempInt & 0xFF;         // Temperatur LSB
  appData[5] = humInt;                 // Luftfeuchtigkeit
  appData[6] = (pressureInt >> 8) & 0xFF; // Luftdruck MSB
  appData[7] = pressureInt & 0xFF;     // Luftdruck LSB
  appData[8] = (windInt >> 8) & 0xFF;  // Windgeschwindigkeit MSB
  appData[9] = windInt & 0xFF;         // Windgeschwindigkeit LSB
  appData[10] = (rainInt >> 8) & 0xFF; // Regenmenge MSB
  appData[11] = rainInt & 0xFF;        // Regenmenge LSB
  appData[12] = (hourlyRainInt >> 8) & 0xFF; // stündliche Regenmenge MSB
  appData[13] = hourlyRainInt & 0xFF;        // stündliche Regenmenge LSB


  // CRC8 über die ersten 7 Bytes berechnen und ans Ende schreiben
  appData[14] = crc8_le(0, appData, 8);

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

  batteryHandler.setup();
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
