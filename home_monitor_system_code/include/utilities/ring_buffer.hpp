#pragma once
#include <stdint.h>

template <typename T, int32_t N = 10>
struct RingBuffer
{
    uint32_t beginIndex = 0;
    uint32_t endIndex = 0;
    int32_t bufferSize = N + 1;
    T bufferData[N + 1];

    int pushData(T& data)
    {
        if((beginIndex + 1) % bufferSize == endIndex)
            return -1; // Full buffer

        bufferData[beginIndex] = data;
        beginIndex = (beginIndex + 1) % bufferSize;

        return 0;
    }

    int popData(T& data)
    {
        if(beginIndex == endIndex)
            return -1; // Empty buffer
        
        data = bufferData[endIndex];
        endIndex = (endIndex + 1) % bufferSize;

        return 0;
    }

    int32_t getDataCount()
    {
        if((beginIndex + 1) % bufferSize == endIndex)
            return bufferSize - 1; // Full buffer
        
        if(beginIndex >= endIndex)
            return beginIndex - endIndex;

        return bufferSize - endIndex + beginIndex;
    }
};

