#include "radio/radio_ll.hpp"
#include "radio/radio.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

constexpr const char* logTag = "RadioTask";

void radioTask(void* arg)
{
    auto taskArg = static_cast<RadioTaskArg*>(arg);
    radioTaskHandle = xTaskGetCurrentTaskHandle();

    assert(taskArg->packetToProcessQueue != nullptr);
    assert(taskArg->toRadioTaskQueue != nullptr);

    ESP_LOGI(logTag, "Started!");

    TimerHandle_t waitTimer = xTimerCreate("WaitTimer", pdMS_TO_TICKS(1000), pdTRUE, nullptr, [](TimerHandle_t xTimer){
        // Just a timer to wait before sending next packet
        notifyRadioTask(RadioTaskNotifyBits::PacketToSent);
    });

    while (true)
    {
        uint32_t notifValue = 0;
        if(xTaskNotifyWait(0, 0xffffffff, &notifValue, portMAX_DELAY) != pdTRUE)
            continue;
        ESP_LOGI(logTag, "Notification received: 0x%lx", notifValue);

        if(notifValue & RadioTaskNotifyBits::NewDataReceived)
        {
            processIrqStateNRF24(taskArg->packetToProcessQueue);
        }
        if(notifValue & RadioTaskNotifyBits::PacketToSent)
        {
            static TickType_t lastSendTime = xTaskGetTickCount();
            /// Have to wait 
            if(xTaskGetTickCount() < lastSendTime + pdMS_TO_TICKS(10000))
            {
                if(xTimerIsTimerActive(waitTimer) == pdFALSE)
                {
                    xTimerStart(waitTimer, 0);
                    ESP_LOGI(logTag, "Waiting before sending next packet...");
                }
                /// Run wait fun
                continue;
            }
            xTimerStop(waitTimer, 0);
            // if(xTimerIsTimerActive(waitTimer) != pdFALSE)
            // {
            //     xTimerStop(waitTimer, 0);
            //     ESP_LOGI(logTag, "Wait timer stopped, sending next packet...");
            // }

            ESP_LOGI(logTag, "Processing packets to send");
            processSensorSends(taskArg->toRadioTaskQueue);
            if(xQueueIsQueueEmptyFromISR(taskArg->toRadioTaskQueue) == pdFALSE)
            {
                ESP_LOGI(logTag, "More packets to send, waiting before sending next packet...");
                xTimerStart(waitTimer, 0);
            }
            lastSendTime = xTaskGetTickCount();
        }
        portYIELD();
    }
}