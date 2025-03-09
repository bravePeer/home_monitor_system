#if defined(RP2040)
#include <stdint.h>
#include <pico/time.h>

uint64_t getTime()
{
    return get_absolute_time();
}


#endif