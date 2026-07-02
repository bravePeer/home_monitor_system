#include <cstdint>
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "esp_http_client.h"
#include "http_client/http_client.hpp"

#if !defined(RECEIVER_HMS_SERVER_API_URL)
#warning RECEIVER_HMS_SERVER_API_URL is not defined! Using default value: localhost:8000
#define RECEIVER_HMS_SERVER_API_URL "http://127.0.0.1:8000"
#endif

constexpr const char* httpHmsApiUrl = RECEIVER_HMS_SERVER_API_URL;
constexpr const char* httpHmsApiUrlSensorAddData = RECEIVER_HMS_SERVER_API_URL "/sensor/data/create";

esp_err_t httpEventHandler(esp_http_client_event_t* event)
{
    return ESP_OK;
}

void testRequest()
{
    esp_http_client_config_t config = {};
    config.url = httpHmsApiUrl;
    config.event_handler = httpEventHandler;
    

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    if(err == ESP_OK)
    {
        ESP_LOGI("HTTP Client", "Status = %d, content_length = %lld",
            esp_http_client_get_status_code(client),
            esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE("HTTP Client", "Error perform http request: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void httpRequestGet()
{
    esp_http_client_config_t config = {};
    config.url = httpHmsApiUrl;
    config.event_handler = httpEventHandler;
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    if(err == ESP_OK)
    {
        ESP_LOGI("HTTP Client", "Status = %d, content_length = %lld",
            esp_http_client_get_status_code(client),
            esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE("HTTP Client", "Error perform http request: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void httpRequestPostSensorData(HttpClientTaskData* data)
{
    esp_http_client_config_t config = {};
    config.url = httpHmsApiUrlSensorAddData;
    config.event_handler = httpEventHandler;
    config.method = esp_http_client_method_t::HTTP_METHOD_POST;
    
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    
    esp_err_t err = esp_http_client_perform(client);
    if(err == ESP_OK)
    {
        ESP_LOGI("HTTP Client", "Status = %d, content_length = %lld",
            esp_http_client_get_status_code(client),
            esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE("HTTP Client", "Error perform http request: %s", esp_err_to_name(err));
    }

    const char *post_data = "{\"field1\":\"value1\"}";
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    err = esp_http_client_perform(client);
    if (err == ESP_OK) 
    {
        ESP_LOGI("HTTP Client", "HTTP POST Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } 
    else 
    {
        ESP_LOGE("HTTP Client", "HTTP POST request failed: %s", esp_err_to_name(err));
    }


    esp_http_client_cleanup(client);
}
