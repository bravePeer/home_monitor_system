#include "platform_time.hpp"
extern "C"
{
    #include "avr/io.h"
    #include "avr/interrupt.h"
    #include "avr/sleep.h"
    #include "util/delay.h"
    #include "avr/wdt.h"
    // #include "util/delay.h"
}

// #include "sensor_attiny2313/sleep.hpp"

// With classic delay
// RAM:   [===       ]  32.0% (used 41 bytes from 128 bytes)
// Flash: [==========]  99.9% (used 2046 bytes from 2048 bytes)
// With interrupt delay (delay is approximated)
// RAM:   [===       ]  32.0% (used 41 bytes from 128 bytes)
// Flash: [==========]  98.5% (used 2018 bytes from 2048 bytes)
// Pros:
// -> gain 28 bytes
// Cons:
// -> uses timer
// -> depentend of clock config
// -> possible large measurement error


inline void timerDelay(uint32_t val)
{
    // TCCR1B = 0;
    // TCNT1 = 0;

    // Load values to OCR1A
    OCR1A = static_cast<uint16_t>(val / 128);

    TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10); // Set prescaler to CLK/1024 and start timer, set CTC mode

    while (!(TIFR & (1<<OCF1A)))
    { /* Do nothing */ }
    
    TCCR1B = 0; // Stop timer
    TIFR |= (1<<OCF1A); // Clear flag
}

// inline uint16_t sleepCounter2 = 0;

extern void sleep(uint16_t);

void delayMs(uint32_t duration)
{
    // timerDelay(duration * 1000);
    // timerDelay(duration << 10);

    // real 250ms
    
    duration = (duration > 250) ? (duration / 250) + 1 : 1;
    sleep(duration);
    // uint16_t sleepCounter2 = 0;
    // do
    // {
    //     wdt_reset();
    //     WDTCR |= (1<<WDCE) | (1<<WDE); // Enable interrupt mode
    //     WDTCR = (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0); // 8s
        
    //     sleep_enable();
    //     sei();
    //     sleep_cpu();
    //     sleep_disable();
    //     sei();
    //     sleepCounter2++;

    // } while (sleepCounter2 > duration);
}

void delayUs(uint32_t duration)
{
    timerDelay(duration);
}