#pragma once
#include <cstring>
#include "sensor/sensor/sensor_common.hpp"
#include "utilities.hpp"

/// @brief 
/// @details Generated structure:
/// {
///     temperature: val,
///     humidity: val,
///     pressure: val,
/// }
/// @param data 
/// @param buffer 
/// @return 
inline int sensorNewestDataToJSON(sensor::SensorType& sensorType, RingBuffer<sensor::SensorSingleData, SensorDataCount>& datas, char* buffer)
{
    switch (sensorType)
    {
    case sensor::SensorType::SimpleWeatherSensorBMP280:
    {
        sensor::SensorSingleData temperature;
        sensor::SensorSingleData pressure;
        temperature.dataType = sensor::SensorSingleDataType::Unknown;
        pressure.dataType = sensor::SensorSingleDataType::Unknown;

        for (int32_t i = 0; i < datas.size(); i++)
        {
            sensor::SensorSingleData data;
            datas.peek(data, i);
            if(data.dataType == sensor::SensorSingleDataType::Temperature && temperature.dataType == sensor::SensorSingleDataType::Unknown)
                temperature = data;
            else if(data.dataType == sensor::SensorSingleDataType::Pressure && pressure.dataType == sensor::SensorSingleDataType::Unknown)
                pressure = data;

            if(temperature.dataType != sensor::SensorSingleDataType::Unknown && pressure.dataType != sensor::SensorSingleDataType::Unknown)
                break;
        }

        return sprintf(buffer, R"({"temperature":%ld, "pressure":%ld})", temperature.temperature, pressure.pressure);
    }
    case sensor::SensorType::SimpleWeatherSensorBME280:
    {
        sensor::SensorSingleData temperature;
        sensor::SensorSingleData pressure;
        sensor::SensorSingleData humidity;
        sensor::SensorSingleData voltage;
        temperature.dataType = sensor::SensorSingleDataType::Unknown;
        pressure.dataType = sensor::SensorSingleDataType::Unknown;
        humidity.dataType = sensor::SensorSingleDataType::Unknown;
        voltage.dataType = sensor::SensorSingleDataType::Unknown;

        for (int32_t i = 0; i < datas.size(); i++)
        {
            sensor::SensorSingleData data;
            datas.peek(data, i);
            if(data.dataType == sensor::SensorSingleDataType::Temperature && temperature.dataType == sensor::SensorSingleDataType::Unknown)
                temperature = data;
            else if(data.dataType == sensor::SensorSingleDataType::Pressure && pressure.dataType == sensor::SensorSingleDataType::Unknown)
                pressure = data;
            else if(data.dataType == sensor::SensorSingleDataType::Humidity && humidity.dataType == sensor::SensorSingleDataType::Unknown)
                humidity = data;
            else if(data.dataType == sensor::SensorSingleDataType::Voltage && voltage.dataType == sensor::SensorSingleDataType::Unknown)
                voltage = data;

            if(temperature.dataType != sensor::SensorSingleDataType::Unknown 
                && pressure.dataType != sensor::SensorSingleDataType::Unknown
                && humidity.dataType != sensor::SensorSingleDataType::Unknown
                && voltage.dataType != sensor::SensorSingleDataType::Unknown)
                break;
        }

        return sprintf(buffer, R"({"temperature":%ld, "pressure":%ld, "humidity":%d, "voltage":%d})", temperature.temperature, pressure.pressure, humidity.humidity, voltage.voltage);
    }
    default:
        break;
    
    }
    return -1;
}

/// @brief Generates JSON with sensor basic informations, no adds \0 at end
/// @details Genarated structure:
/// {
///   "sensor_id":"xxxxxxxx", 
///   "soft_ver":"xxxxxxxx", 
///   "hard_ver":"xxxxxxxx", 
///   "sensor_type":"xxxxxxxx", 
///   "dataPMaxCount":"xx",
///   "calibPMaxCount":"xx", 
///   "radio_addr":"xxxxxxxxxx", 
///   "initTime":"xxxxxxxxxxxxxxxx"
/// }
/// @param sensorInfo 
/// @return Copied bytes count
inline int sensorInfoToJSON(sensor::SensorInfo& sensorInfo, char* buffer)
{
    int i = 0;
    strncpy(buffer, R"({"sensor_id":"xxxxxxxx","soft_ver":"xxxxxxxx","hard_ver":"xxxxxxxx","sensor_type":"xxxxxxxx","dataPMaxCount":"xx","calibPMaxCount":"xx","radio_addr":"xxxxxxxxxx","initTime":"xxxxxxxxxxxxxxxx"})", 193);
    i += 14;
    valueToStrNT(sensorInfo.identifier, &buffer[i]);
    i += 22;
    valueToStrNT(sensorInfo.softwareVersion, &buffer[i]);
    i += 22;
    valueToStrNT(sensorInfo.hardwareVersion, &buffer[i]);
    i += 25;
    valueToStrNT(static_cast<uint32_t>(sensorInfo.sensorType), &buffer[i]);
    i += 27;
    valueToStrNT(sensorInfo.dataPacketsMaxCount, &buffer[i]);
    i += 22;
    valueToStrNT(sensorInfo.calibrationPacketsMaxCount, &buffer[i]);
    i += 18;
    for (int_fast8_t j = 0; j < 5; j++)
    {
        valueToStrNT(sensorInfo.address[j], &buffer[i]);
        i+=2;
    }
    i += 16 - 2;
    valueToStrNT(sensorInfo.initializationTime, &buffer[i]);
    i += 18;
    return i;
}

/// @brief 
/// {
///   "sensor_id":"xxxxxxxx"
/// }
/// @param buf 
/// @param bufSize 
/// @return -1 error
inline uint32_t sensorIdJSONToInt(char* buf, int bufSize)
{
    if(strncmp(R"({"sensor_id":")", buf, 14) != 0)
        return -1;

    uint32_t id = 0;
    int index = 14;
    strToValue(&buf[index], &id);
    return id;
}