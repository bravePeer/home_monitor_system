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

uint8_t readByteEEPROM(uint8_t address)
{
    while(EECR & (1 << EEPE))
    { /* Poll EEPE bit */ }
    EEAR = address;
    EECR |= (1 << EERE);
    return EEDR;
}

void readBytesEEPORM(uint8_t address, uint8_t* dataPtr, uint8_t dataLen)
{
    while(dataLen--)
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

uint32_t readEEPROM(uint8_t address)
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
