#include "RainHandler.hpp"

#define rainPin 7   // Rain gauge pin

RainHandler rainHandler;

RTC_DATA_ATTR int rainCnt = 0;  // Global variable for rain impulses
unsigned long lastRainTipTime = 0; // Time of last rain tip
const unsigned long debounceInterval = 500; // Debounce interval for rain gauge


// Function to count the rain tips ///////////////////////////////////////
void RainHandler::rainCounter(){
  delay(100);
    rainCnt++;
}

// Setup rain gauge ////////////////////////////////////////////////////////////
void RainHandler::setup(){
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), rainCounter, FALLING);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)rainPin, 0);
}

// Function to count the rain tips /////////////////////////////////////////////


int RainHandler::getRainCnt(){
  return rainCnt;
}

void RainHandler::resetRainCnt(){
  rainCnt = 0;
}


extern RainHandler rainHandler;