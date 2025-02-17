#include "RainHandler.hpp"

#define rainPin 7   // Rain gauge pin

RainHandler rainHandler;

RTC_DATA_ATTR int rainCnt = 0;  // Global variable for rain impulses
unsigned long lastRainTipTime = 0; // Time of last rain tip
const unsigned long debounceInterval = 500; // Debounce interval for rain gauge

// Function to count the rain tips (ISR) ///////////////////////////////////////
void IRAM_ATTR RainHandler::rainCounterISR(){
  rainHandler.rainCounter();
}

// Setup rain gauge ////////////////////////////////////////////////////////////
void RainHandler::setup(){
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), rainCounterISR, FALLING);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)rainPin, 0);
}

// Function to count the rain tips /////////////////////////////////////////////
void RainHandler::rainCounter(){
  unsigned long currentMillis = millis();
  if (currentMillis - lastRainTipTime >= debounceInterval) {
    rainCnt++;
    lastRainTipTime = currentMillis;
  }
}

int RainHandler::getRainCnt(){
  return rainCnt;
}

void RainHandler::resetRainCnt(){
  rainCnt = 0;
}

// Handle wakeup by rain gauge /////////////////////////////////////////////////
void RainHandler::handleWakeup(){
  if (digitalRead(rainPin) == LOW) {
    rainCnt++;
  }
}

extern RainHandler rainHandler;