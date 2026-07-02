#pragma once
#include <cstdint>

namespace ledStatus
{
    enum class LedColor: uint8_t
    {
        Red,
        Green
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

    void initLedStatus();

    inline void blinkLed(const LedColor led, const uint8_t times);

    inline void blinkLed(const LedColor led, const BlinkCount times);

}