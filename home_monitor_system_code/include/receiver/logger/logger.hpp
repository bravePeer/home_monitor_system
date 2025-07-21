#pragma once
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <string>
#include <cstdarg>
#include "rp2040/config.hpp"

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

inline void log(const char* who, LogLevel logLevel, const char* info, ...)
{
    char buffer[2048]{0};
    
    sprintf(buffer, "%10lu [ %s ] [ %s ] %s \n", 
        static_cast<uint32_t>(get_absolute_time()),
        logLevelToStr(logLevel),
        who,
        info
    );
    
    va_list args;
    va_start(args, info);

    char buffer2[2048]{0};
    vsprintf(buffer2, buffer, args);
    va_end(args);

    uart_puts(loggerUART, buffer2);
}

inline void initSerialLogger()
{
    gpio_set_function(loggerSerialTx, UART_FUNCSEL_NUM(loggerUART, 0));
    gpio_set_function(loggerSerialRx, UART_FUNCSEL_NUM(loggerUART, 1));
    uart_init(loggerUART, 115200);
    log("Logger", LogLevel::Info, "Logger initialized");
}
