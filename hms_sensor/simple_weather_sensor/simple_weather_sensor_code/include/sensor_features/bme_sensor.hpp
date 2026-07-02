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

/// @brief Initialize BME sensor
void initMeasurementSensor();

/// @brief Start and wait to end of measurements of BME sensor
void doMeasurements();
