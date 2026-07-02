#pragma once
#include <cstdint>

constexpr const char* httpClientTaskTag = "HttpClientTask";

constexpr uint32_t httpClientTaskQueueDataInSize = 10;

enum class HttpClientTaskCommand: uint32_t
{
    TestRequest = 0,
    SendData = 1,
    SendSensorData = 2,
};

/// @brief Data send to queue for http client task
struct HttpClientTaskData
{
    HttpClientTaskCommand command;

    struct
    {
        uint32_t sensorId;
        uint32_t sensorType;
        uint32_t jsonDataSize = 0;
        char* jsonData = nullptr;
    } sensorData;

    /// @todo Add response notify
};

void sendToHttpClientTask(HttpClientTaskData* data);

void httpClientTask(void* arg);
