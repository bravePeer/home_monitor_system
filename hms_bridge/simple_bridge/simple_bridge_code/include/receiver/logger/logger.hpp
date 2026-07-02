#pragma once
#include <string>
#include <cstdarg>
#include <cstdint>
#include "general/platform_time.hpp"
#include "general/platform_logger.hpp"
#include "esp_log.h"

#ifndef LOG_LEVEL
#define LOG_LEVEL 4
#endif

enum class LogLevel: uint32_t
{
    None,
    Error,
    Warning,
    Info,
    Debug
};

inline const char* logLevelToStr(LogLevel logLevel)
{
    switch (logLevel)
    {
    case LogLevel::None:
        return "NONE ";
    case LogLevel::Error:
        return "ERROR";
    case LogLevel::Warning:
        return "WARN ";
    case LogLevel::Info:
        return "INFO ";
    case LogLevel::Debug:
        return "DEBUG";
    }
    return "";
}

#define log( who, logLevel, info, ... ) ESP_LOGI(who, info, ##__VA_ARGS__)
// #define log( who, logLevel, info, ... ) ;


inline void log1(const char* who, LogLevel logLevel, const char* info, ...)
{
    // va_list args;
    // va_start(args, info);
    // esp_log_writev(ESP_LOG_INFO, who, info, args);
    // va_end(args);
    return;
    // char buffer[2048]{0};
    
    // sprintf(buffer, "%10lu [ %s ] [ %s ] %s \n", 
    //     static_cast<uint32_t>(getTime()),
    //     logLevelToStr(logLevel),
    //     who,
    //     info
    // );
    
    // va_list args;
    // va_start(args, info);

    // char buffer2[2048]{0};
    // vsprintf(buffer2, buffer, args);
    // va_end(args);

    // loggerWritePlatform(buffer2);
}

inline void initLogger()
{
    loggerInitPlatform();
    log("Logger", LogLevel::Info, "Logger initialized");
}
