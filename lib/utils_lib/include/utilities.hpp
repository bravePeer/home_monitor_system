#pragma once
#include <cstdint>

constexpr void valueToBytes(const int32_t val, uint8_t* bytes)
{
    bytes[0] = val & 0xff;
    bytes[1] = (val >> 8) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 24) & 0xff;
}

constexpr void valueToBytes(const int val, uint8_t* bytes)
{
    valueToBytes(static_cast<int32_t>(val), bytes);
}

constexpr void valueToBytes(const uint32_t val, uint8_t* bytes)
{
    bytes[0] = val & 0xff;
    bytes[1] = (val >> 8) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 24) & 0xff;
}

constexpr void valueToBytes(const uint64_t val, uint8_t* bytes)
{
    bytes[0] = val & 0xff;
    bytes[1] = (val >> 8) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 24) & 0xff;
    bytes[4] = (val >> 32) & 0xff;
    bytes[5] = (val >> 40) & 0xff;
    bytes[6] = (val >> 48) & 0xff;
    bytes[7] = (val >> 56) & 0xff;
}

/// @brief Converts bytes to unsigned 32 bit int
/// @param bytes Bytes in Little Endian (LSB first)
/// @return Unsigned 32 bit int
constexpr uint32_t bytesToUint32(const uint8_t* bytes)
{
    uint32_t val = bytes[3];
    val <<= 8;
    val |= bytes[2];
    val <<= 8;
    val |= bytes[1];
    val <<= 8;
    val |= bytes[0];
    return val;
}

/// @brief Converts bytes to unsigned 32 bit int
/// @param bytes Bytes in Little Endian (LSB first)
/// @return Signed 32 bit int
constexpr int32_t bytesToInt32(const uint8_t* bytes)
{
    return static_cast<int32_t>(bytesToUint32(bytes));
}

/// @brief Converts bytes to uint16
/// @param bytes Bytes in Little Endian (LSB first) @todo this is for sure LSB?
/// @return Unsigned 16 bit int
constexpr uint16_t bytesToUint16(const uint8_t* bytes)
{
    uint16_t val = bytes[1];
    val <<= 8;
    val |= bytes[0];
    return val;
}

/// @brief Converts bytes to int16
/// @param bytes Bytes in Little Endian (LSB first) @todo this is for sure LSB?
/// @return Signed 16 bit int
constexpr int16_t bytesToInt16(const uint8_t* bytes)
{
    return static_cast<int16_t>(bytesToUint16(bytes));
}

/// @brief Converts uint32_t to hex string, without termination (\0)
/// @param value Value to convert
/// @param dest Destination buffer, size should be at least 8 bytes
inline void valueToStrNT(uint32_t value, char* dest)
{
    for (int_fast8_t i = 7; i >= 0; i--)
    {
        uint_fast8_t tmp = value & 0xf;
        if(tmp <= 0x9)
            dest[i] = 48 + tmp;
        else
            dest[i] = 65 + tmp;
        value >>= 4;
    }
}

/// @brief Converts uint32_t to hex string, without termination (\0)
/// @param value Value to convert
/// @param dest Destination buffer, size should be at least 8 bytes
inline void valueToStrNT(uint8_t value, char* dest)
{
    for (int_fast8_t i = 1; i >= 0; i--)
    {
        uint_fast8_t tmp = value & 0xf;
        if(tmp <= 0x9)
            dest[i] = 48 + tmp;
        else
            dest[i] = 65 + tmp;
        value >>= 4;
    }
}

/// @brief Converts uint32_t to hex string, without termination (\0)
/// @param value Value to convert
/// @param dest Destination buffer, size should be at least 8 bytes
inline void valueToStrNT(uint64_t value, char* dest)
{
    for (int_fast8_t i = 15; i >= 0; i--)
    {
        uint_fast8_t tmp = value & 0xf;
        if(tmp <= 0x9)
            dest[i] = '0' + tmp; 
        else 
            dest[i] = 'A' - 0xA + tmp;
        value >>= 4;
    }
}

/// @brief Convert hex string to uint32_t
/// @param src 
/// @param value 
inline void strToValue(char* src, uint32_t* value)
{
    *value = 0;

    for (int_fast8_t i = 0; i < 8; i++)
    {
        *value <<= 4;
        int32_t tmp = src[i] - 'A';
        *value |= tmp < 0 ? static_cast<uint32_t>(src[i] - '0') : static_cast<uint32_t>(tmp + 0xA);
    }
}
