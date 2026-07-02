#pragma once

#if __has_include(<cstdint>)
    #include <cstdint>
#else
    #include <stdint.h>
#endif

/// @brief Ticks from mcu start
/// @return 
uint64_t getTime();

void delayMs(uint32_t duration);

void delayUs(uint32_t duration);

void initTime();