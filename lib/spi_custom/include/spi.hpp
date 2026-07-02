#pragma once
#include <stdint.h>
#include "port_types_def.hpp"

/// @brief Definition of this function is platform dependent
/// @param baud 
void intSPI(uint_fast16_t baud);

/// @brief Definition of this function is platform dependent, should not drive csn pin
/// @param sendBuf 
/// @param receiveBuf 
/// @param cmd 
/// @param len 
/// @return 
uint8_t transmitLowLevelSPI(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len);

inline uint8_t transmitSPI(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len, portType csnPort, pinType csnPin)
{
  *csnPort &= (~(1<<csnPin));
  uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
  *csnPort |= (1<<csnPin);
  return ret;
}