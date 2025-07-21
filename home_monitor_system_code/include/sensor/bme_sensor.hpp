#pragma once
#if __has_include(<cstdint>)
    #include <cstdint>
#else
    #include <stdint.h>
#endif
#include "bme.hpp"

#if defined(USING_SENSOR_BME280)
constexpr bme::BmeType bmeType = bme::BmeType::BME280;
#elif defined(USING_SENSOR_BMP280)
constexpr bme::BmeType bmeType = bme::BmeType::BMP280;
#elif defined(USING_SENSOR_BME680)
constexpr bme::BmeType bmeType = bme::BmeType::BME680;
#else
#error Select used sensor!
#endif

extern void _delay_ms(double __ms);

inline void initMeasurementSensor()
{
    // uint8_t data = (static_cast<uint8_t>(bme::ControlOversamplingTemperature::OversamplingX2) << 5) |
    //                (static_cast<uint8_t>(bme::ControlOversamplingPressure::OversamplingX16) << 2) |
    //                static_cast<uint8_t>(bme::ControlMode::Sleep);

    uint8_t data = (static_cast<uint8_t>(bme::ControlOversamplingTemperature::OversamplingX1) << 5) |
                   (static_cast<uint8_t>(bme::ControlOversamplingPressure::OversamplingX1) << 2) |
                   static_cast<uint8_t>(bme::ControlMode::Sleep);

    bme::writeReg(&data, bme280::MemoryMap::CtrlMeas);
}

inline void doMeasurements()
{
    bme::BmeSelector<bmeType>::startForcedMeasurement();
    
    uint8_t counter = 10;
    do {
        if (counter == 0) 
            break;
        counter--;
        _delay_ms(10);
    } while (bme::BmeSelector<bmeType>::isBusy());
}