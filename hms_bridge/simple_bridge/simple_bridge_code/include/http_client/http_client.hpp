#pragma once
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

#include "http_client/http_client_task.hpp"
esp_err_t httpEventHandler(esp_http_client_event_t* event);

void testRequest();

void httpRequestPostSensorData(HttpClientTaskData* data);

