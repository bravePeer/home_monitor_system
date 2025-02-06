#if defined(RP2040)
#include <stdint.h>

uint64_t getTime()
{
    return get_absolute_time();
}


#endif