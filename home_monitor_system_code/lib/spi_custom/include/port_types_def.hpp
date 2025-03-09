#pragma once
#include <stdint.h>

#if defined(_PORT_TYPE_UINT8)
using pinType = uint8_t;
using portType = volatile uint8_t *;
#elif defined(_PORT_TYPE_UINT32)
using pinType = uint32_t;
using portType = volatile uint32_t *;
#else
#error Define port type
#endif