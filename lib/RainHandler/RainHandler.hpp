#ifndef RAINHANDLER_HPP
#define RAINHANDLER_HPP

#include <Arduino.h>

class RainHandler
{
  public:
    void setup();
    int getRainCnt();
    void resetRainCnt();
    static void rainCounter();
};

extern RainHandler rainHandler;

#endif