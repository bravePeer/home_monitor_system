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
#include "spi.hpp"

const portType BME_CSN_PORT = &PORTD;
const pinType BME_CSN_PIN = PD6;

const portType NRF24_CSN_PORT = &PORTD;
const pinType NRF24_CSN_PIN = PD5;
inline portType NRF24_CE_PORT = &PORTD;
inline pinType NRF24_CE_PIN = PD4;
const portType NRF24_IRQ_PORT = &PORTD;
const pinType NRF24_IRQ_PIN = PD3;


const portType donePort = &PORTB;
const pinType donePin = PB0;

const portType batteryVoltageEnablePort = &PORTB;
const pinType batteryVoltageEnablePin = PB1;
const portType batteryVoltageCsnPort = &PORTB;
const pinType batteryVoltageCsnPin = PB2;

const portType ledDir = &DDRB;
const portType ledPort = &PORTB;
const pinType ledPin0 = PB4;
const pinType ledPin1 = PB3;

const portType buttonPort = &PORTB;
constexpr uint8_t buttonRightPin = PB5;
constexpr uint8_t buttonLeftPin = PB6;

constexpr pinType progMosi = PB5;
constexpr pinType progMiso = PB6;
constexpr pinType progClk = PB7;

/// @brief Initialize uC and ports
inline void init()
{
    DDRB = (1 << ledPin0) 
        | (1 << ledPin1) 
        | (1 << batteryVoltageEnablePin) 
        | (1 << batteryVoltageCsnPin)
        | (1 << donePin)
#if !defined(NO_SLEEP_TIMER)
        | (1 << progMosi)
        | (0 << progMiso)
        | (1 << progClk)
#endif
        ;

    PORTB = (1 << batteryVoltageCsnPin) 
        | (1 << ledPin0) 
        | (1 << ledPin1) 
#if defined(NO_SLEEP_TIMER)
        | (1 << buttonRightPin) 
        | (1 << buttonLeftPin)
#else
        | (1 << progMosi)
        | (0 << progMiso)
        | (1 << progClk)
#endif
        ;

    DDRD = (1 << NRF24_CSN_PIN) | (1 << NRF24_CE_PIN) | (1 << BME_CSN_PIN);
    PORTD = (1 << NRF24_CSN_PIN) | (1 << NRF24_CE_PIN) | (1 << BME_CSN_PIN);
    
    // MCUCR |= (1<<SE); // Enable sleep is set before go sleep
    MCUCR = (1 << ISC11) // Set falling edgne on INT1 (irqPin)
    | (1<<SM1) | (1<<SM0); // Configure sleep mode to Power Down
    
    // Config interrupt
    GIFR = 0xf4;
    GIMSK = (1 << INT1); // Enable INT1 interrupt

    // Disable ADC
    ACSR |= (1 << ACD);
}

#endif