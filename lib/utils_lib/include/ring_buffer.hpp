#pragma once
#include <stdint.h>

template <typename T, int32_t N = 10>
struct RingBuffer
{
    uint32_t beginIndex = 0;
    uint32_t endIndex = 0;
    int32_t bufferSize = N + 1;
    T bufferData[N + 1];

    int push(const T& data)
    {
        if((beginIndex + 1) % bufferSize == endIndex)
            return -1; // Full buffer

        bufferData[beginIndex] = data;
        beginIndex = (beginIndex + 1) % bufferSize;

        return 0;
    }

    int pop(T& data)
    {
        if(beginIndex == endIndex)
            return -1; // Empty buffer
        
        data = bufferData[endIndex];
        endIndex = (endIndex + 1) % bufferSize;

        return 0;
    }

    /// @brief Peek data from buffer without removing it
    /// @param data Reference to data where copy will be stored
    /// @param offset Offset from last element, 0 means last element
    /// @return If success return 0, else -1
    int peek(T& data, uint32_t offset = 0)
    {
        if(beginIndex == endIndex)
            return -1; // Empty buffer
        
        if(offset >= size())
            return -1; // Out of range
        
        uint32_t index = ((endIndex >= beginIndex) && (offset > beginIndex)) ? bufferSize - (offset - beginIndex) : beginIndex - offset;
        
        data = bufferData[index];

        return 0;
    }

    int size() const
    {
        if((beginIndex + 1) % bufferSize == endIndex)
            return bufferSize - 1; // Full buffer
        
        if(beginIndex >= endIndex)
            return beginIndex - endIndex;

        return bufferSize - endIndex + beginIndex;
    }

    void clear()
    {
        beginIndex = 0;
        endIndex = 0;
    }
};

