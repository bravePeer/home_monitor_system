#include "file_storage/file_storage.hpp"

esp_err_t initMemory()
{
    ESP_LOGI("ResultStorage", "Initializing LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = filepath,
        .partition_label = "storage",
        .partition = nullptr,
#ifdef CONFIG_LITTLEFS_SDMMC_SUPPORT
        .sdcard = nullptr,
#endif
        .format_if_mount_failed = true,
        .read_only = false,
        .dont_mount = false,
        .grow_on_mount = false
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("ResultStorage", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("ResultStorage", "Failed to find LittleFS partition");
        } else {
            ESP_LOGE("ResultStorage", "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0;
    size_t used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE("ResultStorage", "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_littlefs_format(conf.partition_label);
    } else {
        ESP_LOGI("ResultStorage", "Partition size: total: %d, used: %d", total, used);
    }

    return ESP_OK;
}

esp_err_t fileChecks()
{
    FILE* file = fopen(filepathSensor, "r");
    if(file == nullptr)
    {
        ESP_LOGW("ResultStorage", "Failed to open sensor file, creating it!");

        file = fopen(filepathSensor, "w");
        if(file == nullptr)
        {
            ESP_LOGE("ResultStorage", "Failed to create sensor file!");
            return ESP_FAIL;
        }
        else
        {
            fclose(file);
        }
    }
    else
    {
        ESP_LOGI("ResultStorage", "Sensor file exists");
        fclose(file);
    }

    file = fopen(filepathResults, "r");
    if(file == nullptr)
    {
        ESP_LOGW("ResultStorage", "Failed to open results file, creating it!");

        file = fopen(filepathResults, "w");
        if(file == nullptr)
        {
            ESP_LOGE("ResultStorage", "Failed to create results file!");
            return ESP_FAIL;
        }
        else
        {
            fclose(file);
        }
    }
    else
    {
        ESP_LOGI("ResultStorage", "Results file exists");
        fclose(file);
    }
    return ESP_OK;
}

void writeSensorToFile(FILE* sensorFile, sensor::Sensor* sensor)
{
    // Identifier; InitTime; SensorType; SoftwareVersion; HardwareVersion; CalibrationData(c0, c1 ...); ResultCols(temperature, pressure, humidity, voltage ...) 
    // fprintf(sensorFile, "%4lx;%llu;%4lx;%4lx;%4lx;", sensor->info.identifier, sensor->info.initializationTime, static_cast<uint32_t>(sensor->info.sensorType), sensor->info.softwareVersion, sensor->info.hardwareVersion);
    
    char buf[512];
    snprintf(buf, sizeof(buf), "%4lx;%llu;%4lx;%4lx;%4lx;", 
        sensor->info.identifier, 
        sensor->info.initializationTime, 
        static_cast<uint32_t>(sensor->info.sensorType), 
        sensor->info.softwareVersion, 
        sensor->info.hardwareVersion
    );
    fputs(buf, sensorFile);
    
    sensor->getCalibrationDataStr(buf, sizeof(buf));
    fputs(buf, sensorFile);
    fputc(';', sensorFile);

    sensor->getResultsColsStr(buf, sizeof(buf));
    fputs(buf, sensorFile);
    fputc('\n', sensorFile);
}

void writeResultToFile(FILE* resultFile, sensor::Sensor* sensor)
{
    // Identifier; ResultCols(temperature, pressure, humidity, voltage ...) 
    // fprintf(resultFile, "%4lx;%llu;", sensor->info.identifier, sensor->lastSendDataTime);
    char buf[512];

    int len = snprintf(buf, sizeof(buf), "%4lx;%llu;", 
        sensor->info.identifier, 
        sensor->lastSendDataTime
    );
    fwrite(buf, 1, len, resultFile);

    len = sensor->getResultsStr(buf, sizeof(buf));
    fwrite(buf, 1, len, resultFile);
    fputc('\n', resultFile);
}

void fileStorageTask(void* arg)
{
    auto taskArg = static_cast<FileStorageTaskArg*>(arg);

    ESP_ERROR_CHECK(initMemory());
    ESP_LOGI("FileStorage", "Started!");
    ESP_ERROR_CHECK(fileChecks());

    // FILE *sensorFile = fopen(filepathSensor, "a");
    // assert(sensorFile != nullptr);

    // FILE *resultFile = fopen(filepathResults, "a");
    // assert(resultFile != nullptr);

    while (true)
    {
        FileStorageQueueData data;
        if(xQueueReceive(taskArg->packetToProcessQueue, &data, portMAX_DELAY) != pdTRUE)
            continue;
        
        if(data.dataType == FileStorageQueueDataType::StoreResult)
        {
            // [ ] Store calculated data to flash or send to server
            FILE *resultFile = fopen(filepathResults, "a");
            log("FileStorage", LogLevel::Info, "Storing calculated data for sensor id: 0x%lx", data.sensorId);
            writeResultToFile(resultFile, data.sensorPtr);
            fclose(resultFile);
        }
        else if(data.dataType == FileStorageQueueDataType::StoreSensor)
        {
            FILE *sensorFile = fopen(filepathSensor, "a");
            log("FileStorage", LogLevel::Info, "Storing sensor info with sensor id: 0x%lx", data.sensorId);
            writeSensorToFile(sensorFile, data.sensorPtr);
            fclose(sensorFile);
        }
        else if(data.dataType == FileStorageQueueDataType::ExecuteAction)
        {
            log("FileStorage", LogLevel::Info, "Executing action");
            if(data.actionCb == nullptr)
            {
                log("FileStorage", LogLevel::Warning, "Action callback is null!");
                continue;
            }
            data.actionCb(data.actionArg);
            xSemaphoreGive(data.actionDoneSemaphore);
        }
    }
}
