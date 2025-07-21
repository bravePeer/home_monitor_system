#pragma once
#include <pico/stdlib.h>

enum class LedState: uint32_t
{
    Off,
    OnRed,
    OnGreen,
    Invert
};

inline void initLedState()
{
    gpio_init(17);
    gpio_init(16);
    gpio_set_dir(17, true);
    gpio_set_dir(16, true);
}

inline void setLedState(LedState ledState)
{
    const uint ledRedAnodePin = 17;
    const uint ledGreenAnodePin = 16;
    // gpio_set_drive_strength(ledGreenAnodePin, gpio_drive_strength::GPIO_DRIVE_STRENGTH_2MA);
    switch (ledState)
    {
    case LedState::Off:
        gpio_put(ledRedAnodePin, false);
        gpio_put(ledGreenAnodePin, false);
        break;
    case LedState::OnRed:
        gpio_put(ledGreenAnodePin, false);
        gpio_put(ledRedAnodePin, true);
        break;
    case LedState::OnGreen:
        gpio_put(ledGreenAnodePin, true);
        gpio_put(ledRedAnodePin, false);
        // gpio_set_drive_strength(ledGreenAnodePin, gpio_drive_strength::GPIO_DRIVE_STRENGTH_4MA);
        break;
    case LedState::Invert:

        break;
    }
}

inline void blinkLed(LedState ledState, int countOfBlinks, int interval_ms = 200)
{
    setLedState(LedState::Off);
    for (int i = 0; i < countOfBlinks; i++)
    {
        setLedState(ledState);
        sleep_ms(interval_ms);
        setLedState(LedState::Off);
        sleep_ms(interval_ms);
    }
}