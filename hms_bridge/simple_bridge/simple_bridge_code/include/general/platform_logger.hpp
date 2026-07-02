#pragma once
#include <cstddef>

void loggerInitPlatform();

void loggerWritePlatform(char* buffer, [[maybe_unused]] size_t len = 0);
