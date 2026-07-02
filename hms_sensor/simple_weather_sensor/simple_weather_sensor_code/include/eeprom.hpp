#pragma once
extern "C"
{
    #include <avr/io.h>
}
#include <stdint.h>

// Structure of EEPROM:
// <rx address 5 B> 
// <sensor identifier 4B> 
// <hardware version 4B> 
// <software version 4B>

enum class EepromMap: uint8_t
{
    RxAddress       = 0x00,
    Identifier      = 0x05,
    SoftwareVersion = 0x09,
    HardwareVersion = 0x0D,
    SensorType      = 0x11,
    DataPacketsMaxCount         = 0x15,
    CalibrationPacketsMaxCount  = 0x16,
    ResistorR1Value = 0x17,
    ResistorR2Value = 0x19,
};

inline uint8_t readByteEEPROM(uint8_t address)
{
    while(EECR & (1 << EEPE))
    { /* Poll EEPE bit */ }
    EEAR = address;
    EECR |= (1 << EERE);
    return EEDR;
}

inline void readBytesEEPORM(uint8_t address, uint8_t* dataPtr, uint8_t size)
{
    while(size--)
    {
        while(EECR & (1 << EEPE))
        { /* Poll EEPE bit */ }
        EEAR = address;
        EECR |= (1 << EERE);
        *dataPtr = EEDR;
        address++;
        dataPtr++;
    }
}

inline void readBytesEEPORM(EepromMap address, uint8_t* dataPtr, uint8_t size)
{
    readBytesEEPORM(static_cast<uint8_t>(address), dataPtr, size);
}

inline uint32_t readEEPROM(uint8_t address)
{
    // uint32_t ret = 0;
    // ret = readByteEEPROM(address);
    // address++;
    // ret = ret << 8;
    // ret |= readByteEEPROM(address);
    // address++;
    // ret = ret << 8;
    // ret |= readByteEEPROM(address);
    // address++;
    // ret = ret << 8;
    // ret |= readByteEEPROM(address);
    // return ret;
    union Word
    {
        uint32_t word;
        uint8_t raw[4];
    };
    
    Word w;
    w.raw[0] = readByteEEPROM(address);
    w.raw[1] = readByteEEPROM(address + 1);
    w.raw[2] = readByteEEPROM(address + 2);
    w.raw[3] = readByteEEPROM(address + 3);
    return w.word;
}
