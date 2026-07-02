#pragma once
extern "C"
{
    #include "avr/io.h"
    #include "avr/interrupt.h"
    #include "avr/sleep.h"
    #include "util/delay.h"
    #include "avr/wdt.h"
}

#include "state.hpp"
#include "nrf24.hpp"

constexpr uint16_t maxSleepCounter = 2400; // ~10min
inline volatile uint16_t sleepCounter = 0;

uint8_t shouldWait = 0;


ISR(WDT_OVERFLOW_vect)
{
    wdt_reset();
    wdt_disable();
}

inline void sleep(uint16_t sleepTimeCouneter)
{
    sleepCounter = 0;
    do
    {
        wdt_reset();
        WDTCR |= (1<<WDCE) | (1<<WDE); // Enable interrupt mode
        WDTCR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0); // 0.25s
        // WDTCR = (1<<WDIE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0); // 8s

        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
        sei();
        sleepCounter++;
        if(shouldWait)
            return;
    } while (sleepCounter < sleepTimeCouneter);
}

// inline void deep_sleep()
// {
//     // BME after Forced mode goes to sleep itself
//     // Turn of nRF24
//     nrf24::setPower(0);
// }