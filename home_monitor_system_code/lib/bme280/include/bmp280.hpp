#pragma once
#include <ctype.h>

namespace bmp280
{
    enum class MemoryMap: uint8_t
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
        Id = 0xD0,
        CalibT1LSB = 0x88, // Begin of calibration data
        CalibT1MSB = 0x89, 
        CalibT2LSB = 0x8A, 
        CalibT2MSB = 0x8B, 
        CalibT3LSB = 0x8C, 
        CalibT3MSB = 0x8D, 
        CalibP1LSB = 0x8E, 
        CalibP1MSB = 0x8F, 
        CalibP2LSB = 0x90, 
        CalibP2MSB = 0x91, 
        CalibP3LSB = 0x92, 
        CalibP3MSB = 0x93, 
        CalibP4LSB = 0x94, 
        CalibP4MSB = 0x95, 
        CalibP5LSB = 0x96, 
        CalibP5MSB = 0x97, 
        CalibP6LSB = 0x98, 
        CalibP6MSB = 0x99, 
        CalibP7LSB = 0x9A, 
        CalibP7MSB = 0x9B, 
        CalibP8LSB = 0x9C, 
        CalibP8MSB = 0x9D, 
        CalibP9LSB = 0x9E, 
        CalibP9MSB = 0x9F, 
        CalibReservedLSB = 0xA0,
        CalibReservedMSB = 0xA1,
    };

    constexpr uint8_t measuringMask = 0x08;
    constexpr uint8_t imUpdateMask = 0x01;
    constexpr uint8_t busyMask = measuringMask | imUpdateMask;

    constexpr uint8_t idValue = 0x58;
    constexpr uint8_t resetValue = 0xB6;

}