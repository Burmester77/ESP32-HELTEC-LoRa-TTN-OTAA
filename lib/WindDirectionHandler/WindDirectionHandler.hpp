#ifndef WINDDIRECTIONHANDLER_HPP
#define WINDDIRECTIONHANDLER_HPP

#include <Arduino.h>

class WindDirectionHandler {
public:
    void setup();
    void readData();
    float getWindDirection();
    String getWindDirectionLabel(float direction);
};

extern WindDirectionHandler windDirectionHandler;

#endif
