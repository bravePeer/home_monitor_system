#if defined(NATIVE)
#include <stdint.h>

// Time is not needed for testing
uint64_t getTime()
{
    return 0;
}

#endif