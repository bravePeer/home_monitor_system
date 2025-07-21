#pragma once
#include <ctype.h>

namespace bme
{
    enum class ControlOversamplingTemperature
    {
        Skipped         = 0b000,
        OversamplingX1  = 0b001,
        OversamplingX2  = 0b010,
        OversamplingX4  = 0b011,
        OversamplingX8  = 0b100,
        OversamplingX16 = 0b101,
        // OversamplingX16 = 0b110,
        // OversamplingX16 = 0b111,
    };

    enum class ControlOversamplingPressure
    {
        Skipped         = 0b000,
        OversamplingX1  = 0b001,
        OversamplingX2  = 0b010,
        OversamplingX4  = 0b011,
        OversamplingX8  = 0b100,
        OversamplingX16 = 0b101,
        // OversamplingX16 = 0b110,
        // OversamplingX16 = 0b111,
    };
    
    enum class ControlOversamplingHumidity
    {
        Skipped         = 0b000,
        OversamplingX1  = 0b001,
        OversamplingX2  = 0b010,
        OversamplingX4  = 0b011,
        OversamplingX8  = 0b100,
        OversamplingX16 = 0b101,
        // OversamplingX16 = 0b110,
        // OversamplingX16 = 0b111,
    };

    enum class ControlMode
    {
        Sleep  = 0b00,
        Forced = 0b01,
        ForcedAlt = 0b10,

        // Not in every sensor
        Normal = 0b11
    };
}