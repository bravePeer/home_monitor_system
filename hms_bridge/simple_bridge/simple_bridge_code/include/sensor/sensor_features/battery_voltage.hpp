#pragma once
#if __has_include(<cstdint>)
    #include <cstdint>
#else
    #include <stdint.h>
#endif

/// @brief Read battery voltage from sensor
/// @param data pointer where data will be stored, 2 bytes
void readBatteryVoltage(uint8_t* data);
