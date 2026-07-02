#include "general/platform_time.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_sntp.h"

uint64_t getTime()
{
    return xTaskGetTickCount();
}

void delayMs(const uint32_t duration)
{
    const TickType_t xDelay = duration / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);
}

void delayUs(uint32_t val) 
{
    ets_delay_us(val);
}

void initTime()
{
    sntp_set_sync_interval(6 * 60 * 60 * 1000);
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
}
