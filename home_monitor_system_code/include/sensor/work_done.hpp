#pragma once
#include "platform/attiny2313/config.hpp"
#include "platform/attiny2313/led_status.hpp"

inline void indicateWorkDone()
{
    blinkLed(LedColor::Green, BlinkCount::WorkDone);
}

inline void setWorkDone()
{
    // *donePort &= ~(1<<donePin);
    // _delay_ms(1);
    *donePort |= (1<<donePin);
    _delay_ms(1);
}