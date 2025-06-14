#pragma once
#ifdef ATTINY2313A
#include <stdint.h>

constexpr uint8_t RxAddressEEPROM = 0;
constexpr uint8_t IdentifierAddressEEPROM = 5;
constexpr uint8_t SoftwareAddressEEPROM = 9;
constexpr uint8_t HardwareAddressEEPROM = 13;

portType BME280_CSN_PORT = &PORTD;
pinType BME280_CSN_PIN = PD6;

portType NRF24_CSN_PORT = &PORTD;
pinType NRF24_CSN_PIN = PD5;
portType NRF24_CE_PORT = &PORTD;
pinType NRF24_CE_PIN = PD4;
portType NRF24_IRQ_PORT = &PORTD;
pinType NRF24_IRQ_PIN = PD3;

portType batteryVoltageEnablePort = &PORTB;
pinType batteryVoltageEnablePin = PB1;
portType batteryVoltagePort = &PORTB;
pinType batteryVoltagePin = PB2;

portType ledDir = &DDRB;
portType ledPort = &PORTB;
const pinType ledPin0 = PB4;
const pinType ledPin1 = PB3;

constexpr uint8_t buttonRightPin = PB0;
// constexpr uint8_t buttonLeftPin = PB1;



#endif