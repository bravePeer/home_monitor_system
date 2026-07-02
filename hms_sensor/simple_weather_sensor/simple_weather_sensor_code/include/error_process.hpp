#pragma once
#include "led_status.hpp"
#include "utils.hpp"

[[noreturn]] void errorState(BlinkCount blinkCount)
{
    while(true)
    {
        blinkLed(LedColor::Red, blinkCount);
        // _delay_ms(1000); // wait 1s between error signals
        delayMs(1000);
    }
}