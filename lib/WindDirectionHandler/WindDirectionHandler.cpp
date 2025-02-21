#include <WindDirectionHandler.hpp>
#include <alog.h>

#define windVanePin 4 // Wind vane pin

// Global instance of WindDirectionHandler
WindDirectionHandler windDirectionHandler;

// ADC values for each wind direction
const int adcValuesPerAngle[16] = {
    2960, 1482, 1693, 281, 310, 211, 652, 439,
    1036, 870, 2339, 2221, 3815, 3154, 3467, 2618
};

// Wind direction angles corresponding to the ADC values
const float windAngles[16] = {
    0.0, 22.5, 45.0, 67.5, 90.0, 112.5, 135.0, 157.5,
    180.0, 202.5, 225.0, 247.5, 270.0, 292.5, 315.0, 337.5
};

// Variable to store wind direction
float windDirection = 0.0;

// Setup wind vane ////////////////////////////////////////////////////////////
void WindDirectionHandler::setup() {
    pinMode(windVanePin, INPUT);
}

// Read wind direction ////////////////////////////////////////////////////////////
void WindDirectionHandler::readData() {
    int adcValueWindVane = analogRead(windVanePin);

    // Find the closest ADC value to the measured value
    int closestIndex = 0;
    int minDifference = abs(adcValueWindVane - adcValuesPerAngle[0]);

    // Iterate through all ADC values and find the closest match
    for (int i = 1; i < 16; i++) {
        int difference = abs(adcValueWindVane - adcValuesPerAngle[i]);
        // If the difference is smaller than the current minimum difference, update the minimum difference and the closest index
        if (difference < minDifference) {
            minDifference = difference;
            closestIndex = i;
        }
    }
    // Calculate the wind direction based on the closest index
    windDirection = windAngles[closestIndex];
    //ALOG_D("ADC Value: %d, Closest Match: %d, Wind Direction: %.1f°", adcValueWindVane, adcValuesPerAngle[closestIndex], windDirection);
}

// Get the wind direction ////////////////////////////////////////////////////////////
float WindDirectionHandler::getWindDirection() {
    return windDirection;
}

String WindDirectionHandler::getWindDirectionLabel(float direction) {
    if (direction == 0.0) return "N";
    else if (direction == 22.5) return "NNE"; 
    else if (direction == 45.0) return "NE";
    else if (direction == 67.5) return "ENE";
    else if (direction == 90.0) return "E";
    else if (direction == 112.5) return "ESE";
    else if (direction == 135.0) return "SE";
    else if (direction == 157.5) return "SSE";
    else if (direction == 180.0) return "S";
    else if (direction == 202.5) return "SSW";
    else if (direction == 225.0) return "SW";
    else if (direction == 247.5) return "WSW";
    else if (direction == 270.0) return "W";
    else if (direction == 292.5) return "WNW";
    else if (direction == 315.0) return "NW";
    else if (direction == 337.5) return "NNW";
    else return "Unknown";
}