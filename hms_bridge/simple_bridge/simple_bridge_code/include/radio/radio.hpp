#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <type_traits>
#include <utility>

enum class RadioTaskNotifyBits : uint32_t
{
    None                = 0x00,
    NewDataReceived     = 0x01,
    PacketToSent        = 0x02
};

inline TaskHandle_t radioTaskHandle = nullptr;

inline void notifyRadioTask(RadioTaskNotifyBits bits)
{
    if(radioTaskHandle == nullptr)
        return;
    
    xTaskNotifyIndexed(radioTaskHandle, 0, static_cast<uint32_t>(bits), eSetBits);
}

inline void notifyRadioTaskIsr(RadioTaskNotifyBits bits, BaseType_t* higherPriorityTaskWoken)
{
    if(radioTaskHandle == nullptr)
        return;
    
    xTaskNotifyIndexedFromISR(radioTaskHandle, 0, static_cast<uint32_t>(bits), eSetBits, higherPriorityTaskWoken);
}

constexpr uint32_t operator& (uint32_t a, RadioTaskNotifyBits b)
{
    return a & std::to_underlying(b);
}

struct RadioTaskArg
{
    QueueHandle_t packetToProcessQueue = nullptr;
    QueueHandle_t toRadioTaskQueue = nullptr;
};

void radioTask(void* arg);
