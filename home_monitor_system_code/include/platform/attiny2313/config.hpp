#pragma once
#ifdef ATTINY2313A
#include <stdint.h>
extern "C"
{
    #include "avr/io.h"
    #include "avr/interrupt.h"
    #include "avr/sleep.h"
    #include "util/delay.h"
}

portType BME_CSN_PORT = &PORTD;
pinType BME_CSN_PIN = PD6;

portType NRF24_CSN_PORT = &PORTD;
pinType NRF24_CSN_PIN = PD5;
portType NRF24_CE_PORT = &PORTD;
pinType NRF24_CE_PIN = PD4;
portType NRF24_IRQ_PORT = &PORTD;
pinType NRF24_IRQ_PIN = PD3;


portType donePort = &DDRB;
const pinType donePin = PB0;

portType batteryVoltageEnablePort = &PORTB;
pinType batteryVoltageEnablePin = PB1;
portType batteryVoltageCsnPort = &PORTB;
pinType batteryVoltageCsnPin = PB2;

portType ledDir = &DDRB;
portType ledPort = &PORTB;
const pinType ledPin0 = PB4;
const pinType ledPin1 = PB3;

portType buttonPort = &PORTB;
constexpr uint8_t buttonRightPin = PB5;
constexpr uint8_t buttonLeftPin = PB6;


/// @brief Initialize uC and ports
inline void init()
{
    DDRB = (1 << donePin) 
        | (1 << ledPin0) 
        | (1 << ledPin1) 
        | (1 << batteryVoltageEnablePin) 
        | (1 << batteryVoltageCsnPin);
    PORTB = (1 << buttonRightPin) | (1<<buttonLeftPin) | (1 << batteryVoltageCsnPin);

    DDRD = (1 << NRF24_CSN_PIN) | (1 << NRF24_CE_PIN) | (1 << BME_CSN_PIN);
    PORTD = (1 << NRF24_CSN_PIN) | (1 << NRF24_CE_PIN) | (1 << BME_CSN_PIN);
    // Config interrupt
    
    MCUCR = (1 << ISC11); // Set falling edgne on INT1 (irqPin)

    // MCUCR |= (1<<SE); // Enable sleep
    // MCUCR |= (1<<SM1) | (1<<SM0); // Configure sleep mode to Power Down
    GIFR = 0xf4;
    GIMSK = (1 << INT1); // Enable INT1 interrupt
}

#endif