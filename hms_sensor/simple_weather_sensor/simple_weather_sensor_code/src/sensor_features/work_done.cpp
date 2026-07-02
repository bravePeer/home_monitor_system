#include "sensor_features/work_done.hpp"
#include "spi.hpp"
#include "config.hpp"
#include "led_status.hpp"

void indicateWorkDone()
{
    blinkLed(LedColor::Green, BlinkCount::WorkDone);
}

void setWorkDone()
{
    // *donePort &= ~(1<<donePin);
    // _delay_ms(1);
    DDRB |= (1<<donePin);
    *donePort |= (1<<donePin);
    _delay_ms(2);
    *donePort &= ~(1<<donePin);
    // delayMs(1);
}