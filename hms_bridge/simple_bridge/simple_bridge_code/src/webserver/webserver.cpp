#include "receiver/webserver/webserver.hpp"
#include <esp_http_server.h>
#include <string>
#include "sensor/sensor/sensor.hpp"
#include "utilities.hpp"
#include "receiver/logger/logger.hpp"
#include "receiver/webserver/json_utilities.hpp"
#include "esp_log.h"

static char* data;
static size_t dataSize = 0;

constexpr size_t maxChunkSize = 1024;

struct ActionCbArg
{
    const char* fileName;
    char* data;
    size_t offset;
    size_t dataSize;
};

int fileStorageActionReadChunk(void* arg)
{
    ActionCbArg* actionArg = static_cast<ActionCbArg*>(arg);

    FILE* file = fopen(actionArg->fileName, "r");
    if(file == nullptr)
        return ESP_FAIL;

    if(actionArg->offset > 0)
    {
        fseek(file, actionArg->offset, SEEK_SET);
    }
    actionArg->dataSize = fread(actionArg->data, sizeof(char), maxChunkSize, file);
    
    fclose(file);
    return 0;
}

esp_err_t getRoot(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html");
    httpd_resp_set_hdr(r, "Content-Encoding", "gzip");

    FileStorageQueueData indexFile;
    indexFile.dataType = FileStorageQueueDataType::ExecuteAction;
    indexFile.actionCb = [](void* arg)->int {
        ESP_LOGI("Webserver", "Index file sent to client");

        FILE* file = fopen("/littlefs/index.html.gz", "rb");
        if(file == nullptr)
            return ESP_FAIL;
        dataSize = fread(arg, sizeof(char), 4096, file);
        fclose(file);

        return 0;
    };
    indexFile.actionArg = data;
    indexFile.actionDoneSemaphore = xSemaphoreCreateBinary();

    if(auto ctx = static_cast<WebserverUriArg*>(r->user_ctx); xQueueSend(ctx->fileStorageQueue, &indexFile, 100) != pdTRUE)
    {
        ESP_LOGE("Webserver", "Timeout Failed to send index file data to queue!");
        return httpd_resp_send_500(r);
    }
    xSemaphoreTake(indexFile.actionDoneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(indexFile.actionDoneSemaphore);

    return httpd_resp_send(r, data, dataSize);
}

// httpd_uri_t uriRoot = {
//     .uri = "/",
//     .method = HTTP_GET,
//     .handler = getRoot,
//     .user_ctx = nullptr
// };

esp_err_t getFavicon(httpd_req_t *r)
{
    httpd_resp_set_type(r, "image/x-icon");
    httpd_resp_set_hdr(r, "Content-Encoding", "gzip");

    FileStorageQueueData indexFile;
    indexFile.dataType = FileStorageQueueDataType::ExecuteAction;
    indexFile.actionCb = [](void* arg)->int {
        ESP_LOGI("Webserver", "Index file sent to client");

        FILE* file = fopen("/littlefs/favicon.ico.gz", "rb");
        if(file == nullptr)
            return ESP_FAIL;
        dataSize = fread(arg, sizeof(char), 4096, file);
        fclose(file);

        return 0;
    };
    indexFile.actionArg = data;
    indexFile.actionDoneSemaphore = xSemaphoreCreateBinary();

    if(auto ctx = static_cast<WebserverUriArg*>(r->user_ctx); xQueueSend(ctx->fileStorageQueue, &indexFile, 100) != pdTRUE)
    {
        ESP_LOGE("Webserver", "Timeout Failed to send index file data to queue!");
        return httpd_resp_send_500(r);
    }
    xSemaphoreTake(indexFile.actionDoneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(indexFile.actionDoneSemaphore);

    return httpd_resp_send(r, data, dataSize);
}

// httpd_uri_t uriFavicon {
//     .uri = "/favicon.ico",
//     .method = HTTP_GET,
//     .handler = getFavicon,
//     .user_ctx = nullptr
// };

esp_err_t getSensorFile(httpd_req_t* r)
{
    httpd_resp_set_type(r, "text/html");
    // httpd_resp_set_hdr(r, "Content-Encoding", "gzip");

    char a[1024];
    for (size_t i = 0; i < sizeof(a); i++)
        a[i] = (rand() % ('z' - 'a')) + 'a';

    httpd_resp_send_chunk(r, a, sizeof(a)); 

    // vTaskDelay(pdMS_TO_TICKS(1000));

    for (size_t i = 0; i < sizeof(a); i++)
        a[i] = (rand() % ('z' - 'a')) + 'a';
    httpd_resp_send_chunk(r, a, sizeof(a));

    // vTaskDelay(pdMS_TO_TICKS(1000));

    httpd_resp_send_chunk(r, nullptr, 0); 

    return ESP_OK;

    ActionCbArg arg = {
        .data = data,
        .offset = 0,
        .dataSize = 0
    };


    FileStorageQueueData file;
    file.dataType = FileStorageQueueDataType::ExecuteAction;
    file.actionCb = [](void* arg)->int {
        ESP_LOGI("Webserver", "Index file sent to client");

        FILE* file = fopen("/littlefs/sensor.csv", "r");
        if(file == nullptr)
            return ESP_FAIL;

        fseek(file, 0, SEEK_END);
        int pos = ftell(file);
        pos = pos > maxChunkSize ? maxChunkSize : pos;
        fseek(file, -1 * pos, SEEK_END);
        

        dataSize = fread(arg, sizeof(char), pos, file);
        fclose(file);

        return 0;
    };
    file.actionArg = &arg;
    file.actionDoneSemaphore = xSemaphoreCreateBinary();



    if(auto ctx = static_cast<WebserverUriArg*>(r->user_ctx); xQueueSend(ctx->fileStorageQueue, &file, 100) != pdTRUE)
    {
        ESP_LOGE("Webserver", "Timeout Failed to send index file data to queue!");
        return httpd_resp_send_500(r);
    }
    xSemaphoreTake(file.actionDoneSemaphore, portMAX_DELAY);


    vSemaphoreDelete(file.actionDoneSemaphore);

    return httpd_resp_send(r, data, dataSize);
}


esp_err_t getSensorsCount(httpd_req_t *r)
{
    char resp[28];
    sprintf(resp, "{\"sensors_count\":%d}", sensor::knownSensors.size());
    httpd_resp_set_type(r, "application/json");
    
    return httpd_resp_send(r, resp, HTTPD_RESP_USE_STRLEN);
}

httpd_uri_t uriGetSensorsCount {
    .uri = "/api/get_sensors_count",
    .method = HTTP_POST,
    .handler = getSensorsCount,
    .user_ctx = nullptr
};

/// @brief 
/// {
///   "sensor_ids": ["id0_in_hex", "id1_in_hex", ...]
/// }
/// @param r 
/// @return 
esp_err_t getSensorsId(httpd_req_t *r)
{
    int respSize = 18 + ((sensor::knownSensors.size() > 0) ? 11 * sensor::knownSensors.size() - 1 : 0);
    char* resp = new char[respSize];
    
    strncpy(resp, "{\"sensors_ids\":[", 17);
    int index = 16;

    sensor::Sensor* sen = nullptr;
    for (int i = 0; i < sensor::knownSensors.size(); i++)
    {
        resp[index] = '\"';
        index++;
        sensor::knownSensors.valueAt(i, sen);
        if (sen == nullptr)
        {
            delete[] resp;
            return ESP_FAIL;
        }
        valueToStrNT(sen->info.identifier, &resp[index]);
        index += 8;
        resp[index] = '\"';
        index++;
        resp[index] = ',';
        index++;
    }

    resp[respSize - 2] = ']';
    resp[respSize - 1] = '}';

    httpd_resp_set_type(r, "application/json");
    esp_err_t ret = httpd_resp_send(r, resp, respSize);
    delete[] resp;

    return ret;
}

httpd_uri_t uriGetSensorsId {
    .uri = "/api/get_sensors_ids",
    .method = HTTP_POST,
    .handler = getSensorsId,
    .user_ctx = nullptr
};

esp_err_t getSensor(httpd_req_t* r)
{
    return ESP_OK;
}

httpd_uri_t uriGetSensor {
    .uri = "/api/get_sensor",
    .method = HTTP_POST,
    .handler = getSensor,
    .user_ctx = nullptr
};

esp_err_t getSensorData(httpd_req_t* r)
{
    int respSize = 0;
    // char* resp = new char[respSize];
    char resp[200];

    // r->content_len;
    int recvCount = httpd_req_recv(r, resp, 200);
    ESP_LOGI("Webserver", "Response: '%s'", resp);
    if(recvCount <= 0)
    {
        ESP_LOGE("Webserver", "Response count error: %d", recvCount);
        httpd_resp_set_status(r, HTTPD_400);
        return httpd_resp_send(r, nullptr, 0);
    }

    uint32_t sensorId = sensorIdJSONToInt(resp, recvCount);
    sensor::Sensor* sensorPtr = nullptr;
    int isKnownSensor = sensor::knownSensors.valueByExpression([](sensor::Sensor& refSensor, void* args)->int {
            if(refSensor.info.identifier == *reinterpret_cast<uint32_t*>(args))
                return 0;
            return -1;
    }, sensorPtr, reinterpret_cast<void*>(&sensorId));

    if(isKnownSensor == -1 || sensorPtr == nullptr)
    {
        ESP_LOGE("Webserver", "Sensor not known 0x%x", (int)sensorId);
        httpd_resp_set_status(r, HTTPD_400);
        return httpd_resp_send(r, nullptr, 0);
    }

    if(sensorPtr->status.statusReg.isDataCalculated == 0)
    {
        /// @todo response when data not calculated
    }

    respSize = sensorNewestDataToJSON(sensorPtr->info.sensorType, sensorPtr->calculatedData, resp);
    ESP_LOGI("Webserver0", "json(%d bytes):%s", respSize, resp);
    httpd_resp_set_type(r, "application/json");
    esp_err_t ret = httpd_resp_send(r, resp, respSize);
    // delete[] resp;
    return ret;
}

httpd_uri_t uriGetSensorData {
    .uri = "/api/get_sensor_data",
    .method = HTTP_POST,
    .handler = getSensorData,
    .user_ctx = nullptr
};

/// @brief 
/// {
///   "sensor_info": [{
///     "sensor_id":"id_hex", "soft_ver":"hex", "hard_ver":"hex", "sensor_type":"hex", "dataPMaxCount":"hex"
///     "calibPMaxCount":"hex", "radio_addr":"hexhex", "initTime":"hexhexhex"
///     }, ...]
/// }
/// @param r 
/// @return 
esp_err_t getSensorInfo(httpd_req_t* r)
{
    int respSize = 0;
    // char* resp = new char[respSize];
    char resp[200];

    // r->content_len;
    int recvCount = httpd_req_recv(r, resp, 200);
    ESP_LOGI("Webserver", "Response: '%s'", resp);
    if(recvCount <= 0)
    {
        ESP_LOGE("Webserver", "Response count error: %d", recvCount);
        httpd_resp_set_status(r, HTTPD_400);
        return httpd_resp_send(r, nullptr, 0);
    }
    
    uint32_t sensorId = sensorIdJSONToInt(resp, recvCount);
    sensor::Sensor* sensorPtr = nullptr;
    int isKnownSensor = sensor::knownSensors.valueByExpression([](sensor::Sensor& refSensor, void* args)->int {
            if(refSensor.info.identifier == *reinterpret_cast<uint32_t*>(args))
                return 0;
            return -1;
    }, sensorPtr, reinterpret_cast<void*>(&sensorId));

    if(isKnownSensor == -1 || sensorPtr == nullptr)
    {
        ESP_LOGE("Webserver", "Sensor not known 0x%x", (int)sensorId);
        // log("Webserver", LogLevel::Error, "Sensor not known 0x%x", sensorId);
        httpd_resp_set_status(r, HTTPD_400);
        return httpd_resp_send(r, nullptr, 0);
    }

    if(sensorPtr->status.statusReg.isSensorInfoKnown == 0)
    {
        /// @todo response with sensor info not known
    }

    respSize = sensorInfoToJSON(sensorPtr->info, resp);
    ESP_LOGI("Webserver0", "json(%d bytes):%s", respSize, resp);
    httpd_resp_set_type(r, "application/json");
    esp_err_t ret = httpd_resp_send(r, resp, respSize);
    // delete[] resp;
    return ret;
}

httpd_uri_t uriGetSensorInfo {
    .uri = "/api/get_sensor_info",
    .method = HTTP_POST,
    .handler = getSensorInfo,
    .user_ctx = nullptr
};

esp_err_t getSensorCalibrationData(httpd_req_t* r)
{
    /// @todo
    return ESP_OK;
}

httpd_uri_t uriGetSensorCalibrationData {
    .uri = "/api/get_sensor_calib_data",
    .method = HTTP_POST,
    .handler = getSensorCalibrationData,
    .user_ctx = nullptr
};

esp_err_t getSensorStatus(httpd_req_t* r)
{
    /// @todo
    return ESP_OK;
}

httpd_uri_t uriGetSensorStatus {
    .uri = "/api/get_sensor_last_data",
    .method = HTTP_POST,
    .handler = getSensorStatus,
    .user_ctx = nullptr
};






ErrorCode initWebserver(WebserverUriArg* arg)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = nullptr;

    data = new char[4096];

    if(httpd_start(&server, &config) != ESP_OK)
        return ErrorCode::Fail;

    httpd_uri_t uriRoot = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = getRoot,
        .user_ctx = arg
    };

    httpd_uri_t uriFavicon {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = getFavicon,
        .user_ctx = arg
    };

    httpd_uri_t uriGetSensorFile {
        .uri = "/api/get_sensor_file",
        .method = HTTP_GET,
        .handler = getSensorFile,
        .user_ctx = arg
    };

    httpd_register_uri_handler(server, &uriRoot);
    httpd_register_uri_handler(server, &uriFavicon);
    httpd_register_uri_handler(server, &uriGetSensorFile);
    // httpd_register_uri_handler(server, &uriGetSensorsCount);
    // httpd_register_uri_handler(server, &uriGetSensorsId);
    // httpd_register_uri_handler(server, &uriGetSensorInfo);
    // httpd_register_uri_handler(server, &uriGetSensorData);

    

    return ErrorCode::Ok;
}
