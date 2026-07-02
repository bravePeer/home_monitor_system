#pragma once

#if defined(NRF24_SIMULATOR)

// #include "nrf24_simulator.hpp"
// uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
// {
//     return nrf24::Nrf24::getInstance().transmitSpi(sendBuf, receiveBuf, cmd, len);
// }
#else
// extern uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len);

#endif

#include "nrf24_hardware.hpp"