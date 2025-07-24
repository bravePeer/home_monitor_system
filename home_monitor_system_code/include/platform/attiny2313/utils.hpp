#pragma once
extern "C"
{
    #include "util/delay.h"
}
#include "config.hpp"
#include "spi.hpp"

[[gnu::always_inline]] inline void eDELAY_MS(uint32_t val)
{
    _delay_ms(static_cast<double>(val));
}

[[gnu::always_inline]] inline void eDELAY_US(uint32_t val)
{
    _delay_us(static_cast<double>(val));
}

uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *NRF24_CSN_PORT &= (~(1<<NRF24_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *NRF24_CSN_PORT |= (1<<NRF24_CSN_PIN);
    return ret;
    // return transmitSPI(sendBuf, receiveBuf, cmd, len, NRF24_CSN_PORT, NRF24_CSN_PIN);
}

uint8_t transmitSpiBme280(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *BME_CSN_PORT &= (~(1<<BME_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *BME_CSN_PORT |= (1<<BME_CSN_PIN);
    return ret;
    // return transmitSPI(sendBuf, receiveBuf, cmd, len, NRF24_CSN_PORT, NRF24_CSN_PIN);
}

int8_t isButtonPressed(portType port, uint8_t pin)
{
    if(!(*port & (1<<pin)))
    {
        _delay_ms(10);
        if(!(*port & (1<<pin)))
        {
            return 1;
        }
    }
    return 0;
}
