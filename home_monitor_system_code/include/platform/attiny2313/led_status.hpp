#pragma once
#include <stdint.h>

#include "config.hpp"

// LED Status
// Initializing -> R
// Initialized -> 5 G
// Send -> 3 G
// Received -> 2 G
// 
// Wrong CRC -> 5 R
// NotSend MaxRetr -> 3 R
// 

extern void eDELAY_MS(uint32_t);

inline void turnOffLeds()
{
    *ledPort &= ~((1<<ledPin0) | (1<<ledPin1));
}

inline void turnOnRedLed()
{
    turnOffLeds();
    *ledPort |= (1<<ledPin0);
}

inline void turnOnGreenLed()
{
    turnOffLeds();
    *ledPort |= (1<<ledPin1);
}

enum class LedColor: pinType
{
    Red = ledPin0,
    Green = ledPin1
};

enum class BlinkCount: uint8_t
{
    Initialized = 5,
    Send = 3,
    Received = 2,
    Some = 1,
    WorkDone = 10,

    // Errors count
    Nrf24Error = 2,
    BmeError = 3,
    MaxRetr = 4,
    WrongCRC = 5
};

inline void blinkLed(const LedColor led, const BlinkCount times)
{
    turnOffLeds();
    for (int i = 0; i < static_cast<uint8_t>(times); i++)
    {
        *ledPort |= (1 << static_cast<uint8_t>(led));
        eDELAY_MS(100);
        turnOffLeds();
        eDELAY_MS(100);
    }
}
