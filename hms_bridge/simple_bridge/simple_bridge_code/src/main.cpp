#include <hal/gpio_hal.h>
#include <hal/wdt_hal.h>
#include <nvs_flash.h>
#include <esp_log.h>

#include "receiver/network/network.hpp"
#include "filesystem.hpp"

#if !defined(DISABLE_WEB_SERVER)        
#include "receiver/webserver/webserver.hpp"
#endif

#include "gpio.hpp"
#include "radio/radio.hpp"
#include "radio/radio_ll.hpp"
#include "spi.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "packet_processor/packet_processor.hpp"

#include "file_storage/file_storage.hpp"

#include "http_client/http_client.hpp"
#include "http_client/http_client_task.hpp"

static const char* Tag = "Main";

extern "C" void __attribute__((noreturn)) app_main(void)
{
    if(initNvs() == ErrorCode::Fail)
    {
        ESP_LOGE(Tag, "Memory init error!");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    static FileStorageTaskArg fileStorageTaskArg;
    fileStorageTaskArg.packetToProcessQueue = xQueueCreate(10, sizeof(FileStorageQueueData));

    if(initNetwork() != ErrorCode::Ok)
    {
        ESP_LOGE(Tag, "Network init error!");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(Tag, "Network initialized!");

    ErrorCode ret = connectNetwork();
    if(ret != ErrorCode::Ok)
    {
        ESP_LOGE(Tag, "Network can not connect! ErrorCode: %d", static_cast<int>(ret));
        /// @todo Change to sleep
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(Tag, "Connected to network!");

    
#if !defined(DISABLE_WEB_SERVER)        
    static WebserverUriArg webserverUriArg;
    webserverUriArg.fileStorageQueue = fileStorageTaskArg.packetToProcessQueue;

    if(initWebserver(&webserverUriArg) != ErrorCode::Ok)
    {
        ESP_LOGE(Tag, "Webserver can not be initialized!");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(Tag, "Webserver initialized!");
#endif

    if(initGpio() != ErrorCode::Ok)
    {
        ESP_LOGE(Tag, "GPIO can not be initialized!");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(Tag, "GPIO initialized!");

    intSPI(10000);
    ESP_LOGI(Tag, "SPI initialized!");

    if(initRadio() != 0)
    {
        ESP_LOGE(Tag, "Radio can not be initialized!");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(Tag, "Radio initialized!");




    // ------------------- Test HTTP Client ------------------

    // -------------------------------------------------------


    static RadioTaskArg radioTaskArg;
    static PacketProcessorTaskArg packetProcessorTaskArg;
    packetProcessorTaskArg.packetToProcessQueue = xQueueCreate(20, sizeof(PacketToProcess)); // Direction radioTask -> packetProcessorTask
    packetProcessorTaskArg.toRadioTaskQueue = xQueueCreate(20, sizeof(PacketToProcess)); // Direction packetProcessorTask -> radioTask
    radioTaskArg.packetToProcessQueue = packetProcessorTaskArg.packetToProcessQueue;
    radioTaskArg.toRadioTaskQueue = packetProcessorTaskArg.toRadioTaskQueue;

    
    packetProcessorTaskArg.fileStorageQueue = fileStorageTaskArg.packetToProcessQueue;

    xTaskCreate(httpClientTask, "HttpClientTask", 4096, nullptr, tskIDLE_PRIORITY, nullptr);
    xTaskCreate(radioTask, "RadioTask", 4096, &radioTaskArg, tskIDLE_PRIORITY, nullptr);
    xTaskCreate(packetProcessorTask, "PacketProcessorTask", 4096 * 2, &packetProcessorTaskArg, tskIDLE_PRIORITY, nullptr);
    xTaskCreate(fileStorageTask, "FileStorageTask", 4096, &fileStorageTaskArg, tskIDLE_PRIORITY, nullptr);

    while (true) 
    {
        HttpClientTaskData data;
        data.command = HttpClientTaskCommand::TestRequest;
        sendToHttpClientTask(&data);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY));
    }
}