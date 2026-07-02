#pragma once
#include "general/platform_error.hpp"

ErrorCode initGpio();

ErrorCode setLevel(uint32_t gpioNum, uint32_t state);

extern volatile int32_t irqNrf24Flag;
