#pragma once
#include "spi.h"

#if !defined(BME280_CSN_PORT)
#error Define BME280_CSN_PORT
#endif
#if !defined(BME280_CSN_PIN)
#error Define BME280_CSN_PIN
#endif

namespace bme280
{
  enum class MemoryMap : uint8_t
  {
    HumLsb = 0xFE,
    HumMsb = 0xFD,
    TempXlsb = 0xFC,
    TempLsb = 0xFB,
    TempMsb = 0xFA,
    PressXlsb = 0xF9,
    PressLsb = 0xF8,
    PressMsb = 0xF7,
    Config = 0xF5,
    CtrlMeas = 0xF4,
    Status = 0xF3,
    CtrlHum = 0xF2,
    Reset = 0xE0,
    Id = 0xD0
  };

  uint8_t readId()
  {
    uint8_t data = 0;
    transmitSPI(nullptr, &data, 0xD0, 1, &BME280_CSN_PORT, BME280_CSN_PIN);
    return data;
  }
}
