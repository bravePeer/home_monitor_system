#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor/sensor/sensor.hpp"
#include "esp_littlefs.h"
#include "esp_log.h"

using FileStorageQueueHandle = QueueHandle_t;

const char filepath[] = "/littlefs";
const char filepathSensor[] = "/littlefs/sensor.csv";
const char filepathResults[] = "/littlefs/results.csv";

enum class FileStorageQueueDataType: uint32_t
{
    StoreResult,
    StoreSensor,
    ExecuteAction
};

struct FileStorageQueueData
{
    FileStorageQueueDataType dataType;
    uint32_t sensorId;
    sensor::Sensor* sensorPtr;

    int (*actionCb)(void* arg);
    void* actionArg;    
    SemaphoreHandle_t actionDoneSemaphore;
};

struct FileStorageTaskArg
{
    FileStorageQueueHandle packetToProcessQueue = nullptr;
};

void fileStorageTask(void* arg);
