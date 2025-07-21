#pragma once
#include <stdint.h>


namespace bme680
{
    /// @brief BME680 Memory Map 
    /// @note Register Id and Reset are on Page 0 others on Page 1
    enum class MemoryMap: uint8_t
    {
        Status = 0x73,
        Reset_page0 = 0x60,
        Id_page0 = 0x50,


        //Calibration

        ParT1LSB  = 0xE9,
        ParT1MSB  = 0xEA,
        ParT2LSB  = 0x8A,
        ParT2MSB  = 0x8B,
        ParT3     = 0x8C,

        ParP1LSB  = 0x8E,
        ParP1MSB  = 0x8F,
        ParP2LSB  = 0x90,
        ParP2MSB  = 0x91,
        ParP3     = 0x92,
        ParP4LSB  = 0x94,
        ParP4MSB  = 0x95,
        ParP5LSB  = 0x96,
        ParP5MSB  = 0x97,
        ParP6     = 0x99,
        ParP7     = 0x98,
        ParP8LSB  = 0x9C,
        ParP8MSB  = 0x9D,
        ParP9LSB  = 0x9E,
        ParP9MSB  = 0x9F,
        ParP10    = 0xA0,
        
        // Only <3:0> bits
        ParH1LSB = 0xE2,
        ParH1MSB = 0xE3,
        //Only <7:4> bits
        ParH2LSB = 0xE2,
        ParH2MSB = 0xE1,
        ParH3 = 0xE4,
        ParH4 = 0xE5,
        ParH5 = 0xE6,
        ParH6 = 0xE7,
        ParH7 = 0xE8,

        // Registers Page 1
        Config = 0x75,
        CtrlMeas = 0x74,
        CtrlHum = 0x72,
        CtrlGas1 = 0x71,
        CtrlGas0 = 0x70,
        GasWait9 = 0x6D,
        GasWait8 = 0x6C,
        GasWait7 = 0x6B,
        GasWait6 = 0x6A,
        GasWait5 = 0x69,
        GasWait4 = 0x68,
        GasWait3 = 0x67,
        GasWait2 = 0x66,
        GasWait1 = 0x65,
        GasWait0 = 0x64,
        ResHeat9 = 0x63,
        ResHeat8 = 0x62,
        ResHeat7 = 0x61,
        ResHeat6 = 0x60,
        ResHeat5 = 0x5F,
        ResHeat4 = 0x5E,
        ResHeat3 = 0x5D,
        ResHeat2 = 0x5C,
        ResHeat1 = 0x5B,
        ResHeat0 = 0x5A,
        IdacHeat9 = 0x59,
        IdacHeat8 = 0x58,
        IdacHeat7 = 0x57,
        IdacHeat6 = 0x56,
        IdacHeat5 = 0x55,
        IdacHeat4 = 0x54,
        IdacHeat3 = 0x53,
        IdacHeat2 = 0x52,
        IdacHeat1 = 0x51,
        IdacHeat0 = 0x50,
        GasRLSB = 0x2B,
        GasRMSB = 0x2A,
        HumLSB = 0x26,
        HumMSB = 0x25,
        TempXLSB = 0x24,
        TempLSB = 0x23,
        TempMSB = 0x22,
        PressXLSB = 0x21,
        PressLSB = 0x20,
        PressMSB = 0x1F,
        MeasStatus0 = 0x1D
    };


    constexpr uint8_t idValue = 0x61;
    constexpr uint8_t resetValue = 0xB6;

    constexpr uint8_t measuringMask = 0x20;
    constexpr uint8_t busyMask = measuringMask;

}