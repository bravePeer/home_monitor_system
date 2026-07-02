#include "http_client/http_client_task.hpp"
#include "http_client/http_client.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/// @todo Remove from global scope
static QueueHandle_t httpQueue = nullptr;

void sendToHttpClientTask(HttpClientTaskData* data)
{
    if(httpQueue == nullptr)
    {
        ESP_LOGE(httpClientTaskTag, "Http client task is not initialized!");
        return;
    }
    xQueueSend(httpQueue, data, 0);
}

void httpClientTask(void* arg)
{
    httpQueue = xQueueCreate(httpClientTaskQueueDataInSize, sizeof(HttpClientTaskData));

    while(true)
    {
        HttpClientTaskData data;
        if(xQueueReceive(httpQueue, &data, portMAX_DELAY) == pdFALSE)
        {
            continue;
        }

        switch(data.command)
        {
            case HttpClientTaskCommand::TestRequest:
                testRequest();
                break;
            case HttpClientTaskCommand::SendData:
                /// @todo Implement send data
                break;
            case HttpClientTaskCommand::SendSensorData:
                if(data.sensorData.jsonData == nullptr)
                {
                    ESP_LOGW(httpClientTaskTag, "No data to send! Skipping");
                    break;
                }
                httpRequestPostSensorData(&data);
                delete[] data.sensorData.jsonData;
                break;
            default:
                ESP_LOGW(httpClientTaskTag, "Unknown command received in http client task! Command: %d", static_cast<int>(data.command));
                break;
        }
    }
}
