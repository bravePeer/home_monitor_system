#pragma once
#include "general/platform_error.hpp"
#include "file_storage/file_storage.hpp"

struct WebserverUriArg
{
    FileStorageQueueHandle fileStorageQueue;
};

ErrorCode initWebserver(WebserverUriArg* arg);
