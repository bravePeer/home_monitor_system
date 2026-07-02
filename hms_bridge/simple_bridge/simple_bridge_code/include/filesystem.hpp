#pragma once
#include "general/platform_error.hpp"
#include "nvs_flash.h"
#include "dirent.h"
#include "esp_spiffs.h"

inline ErrorCode initNvs()
{
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if(err != ESP_OK)
        return ErrorCode::Fail;

    return ErrorCode::Ok;
}

// inline ErrorCode initMemory()
// {
//     esp_vfs_spiffs_conf_t config {
//         .base_path = "/spiffs",
//         .partition_label = "spiffs",
//         .max_files = 5,
//         .format_if_mount_failed = false,
//     };

//     esp_err_t err = esp_vfs_spiffs_register(&config);
//     if(err != ESP_OK)
//         return ErrorCode::Fail;

//     if(!esp_spiffs_mounted("spiffs"))
//         return ErrorCode::Fail;

//     err = nvs_flash_init();
//     if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         err = nvs_flash_init();
//     }

//     if(err != ESP_OK)
//         return ErrorCode::Fail;

//     return ErrorCode::Ok;
// }
