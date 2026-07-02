#include "receiver/network/network.hpp"
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <cstring>

static EventGroupHandle_t WifiEventGroup;

#if !defined(RECEIVER_WIFI_SSID)
#warning Wifi SSID not set, using default value: "ssid"
#define RECEIVER_WIFI_SSID "ssid"
#endif
#if !defined(RECEIVER_WIFI_PASSWORD)
#warning Wifi passowrd not set, using default value: "password"
#define RECEIVER_WIFI_PASSWORD "password"
#endif

constexpr EventBits_t WifiConnectedBit = BIT0;
constexpr EventBits_t WifiFailBit = BIT1;

constexpr int MaxConnectionRetry = 10;

static void eventHandler(void* arg, esp_event_base_t eventBase, int32_t eventId, void* eventData)
{
    static int retryCounter = 0;

    if(eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if(eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED)
    {
        if(retryCounter < MaxConnectionRetry)
        {
            esp_wifi_connect();
            retryCounter++;
        }
        else
        {
            xEventGroupSetBits(WifiEventGroup, WifiFailBit);
        }
    }
    else if(eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* eventDataIp = reinterpret_cast<ip_event_got_ip_t*>(eventData);
        ESP_LOGI("Network", "IP: " IPSTR, IP2STR(&eventDataIp->ip_info.ip));
        xEventGroupSetBits(WifiEventGroup, WifiConnectedBit);
        retryCounter = 0;
    }
}

ErrorCode initNetwork()
{
    WifiEventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));

    esp_event_handler_instance_t instanceAnyId;
    esp_event_handler_instance_t instanceGotIp;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, nullptr, &instanceAnyId));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, nullptr, &instanceGotIp));

    // wifi_config_t wifiConfig = {
    //     .sta = {
    //         .ssid = RECEIVER_WIFI_SSID,
    //         .password = RECEIVER_WIFI_PASSWORD,
    //         .threshold = {
    //             .authmode = WIFI_AUTH_WPA2_PSK
    //         }
    //     }  
    // };

    wifi_config_t wifiConfig = {};
    strncpy(reinterpret_cast<char*>(wifiConfig.sta.ssid), RECEIVER_WIFI_SSID, sizeof(wifiConfig.sta.ssid));
    strncpy(reinterpret_cast<char*>(wifiConfig.sta.password), RECEIVER_WIFI_PASSWORD, sizeof(wifiConfig.sta.password));
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ErrorCode::Ok;
}

ErrorCode connectNetwork()
{
    EventBits_t bits = xEventGroupWaitBits(WifiEventGroup, WifiConnectedBit | WifiFailBit, pdFALSE, pdFALSE, portMAX_DELAY);

    ErrorCode ec = ErrorCode::Fail;

    if(bits & WifiConnectedBit)
    {
        ec = ErrorCode::Ok;
    }
    else if(bits & WifiFailBit)
    {
        ec = ErrorCode::Fail;
    }
    
    vEventGroupDelete(WifiEventGroup);
    return ec;
}