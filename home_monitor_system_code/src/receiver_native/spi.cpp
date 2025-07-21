#include <iostream>
#include "spi.hpp"

uint8_t transmitLowLevelSPI(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    std::cout << "SPI Transmit: " << std::hex << static_cast<int>(cmd) << " ";
    for (uint8_t i = 0; i < len; i++)
    {
        std::cout << std::hex << static_cast<int>(sendBuf[i]) << " ";
    }
    std::cout << std::endl;

    return len;
}

uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    return ret;
}