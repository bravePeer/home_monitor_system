#include "general/platform_logger.hpp"
#include <esp_log.h>

void loggerInitPlatform()
{
    // do nothing
}

void loggerWritePlatform(char* buffer, [[maybe_unused]] size_t len)
{
    ESP_LOGI("LOG", "%s", buffer);
}
