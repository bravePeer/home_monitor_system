#pragma once
#include "config.hpp"
#include "spi.hpp"
#include "platform_time.hpp"


inline uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *NRF24_CSN_PORT &= (~(1<<NRF24_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *NRF24_CSN_PORT |= (1<<NRF24_CSN_PIN);
    return ret;
    // return transmitSPI(sendBuf, receiveBuf, cmd, len, NRF24_CSN_PORT, NRF24_CSN_PIN);
}

inline uint8_t transmitSpiBme280(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *BME_CSN_PORT &= (~(1<<BME_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *BME_CSN_PORT |= (1<<BME_CSN_PIN);
    return ret;
    // return transmitSPI(sendBuf, receiveBuf, cmd, len, NRF24_CSN_PORT, NRF24_CSN_PIN);
}

inline int8_t isButtonPressed(portType port, uint8_t pin)
{
    if(!(*port & (1<<pin)))
    {
        delayMs(10);
        if(!(*port & (1<<pin)))
        {
            return 1;
        }
    }
    return 0;
}

inline void setPinCE(uint_fast8_t state)
{
    if(state)
        *NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
    else
        *NRF24_CE_PORT &= (~(1<<NRF24_CE_PIN));
}

