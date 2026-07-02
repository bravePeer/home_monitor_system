#pragma once
#if __has_include(<cstdint>)
    #include <cstdint>
#else
    #include <stdint.h>
#endif

/// @brief Indicate to user end work
void indicateWorkDone();

/// @brief Set specific parameters to end work
void setWorkDone();
