#if defined(WEATHER_RECEIVER)
#pragma once
#include <stdint.h>
#include "sensor/sensor_packet.hpp"
#include "utilities/ring_buffer.hpp"

constexpr unsigned DataBufferSize = 10;

namespace sensor
{
    enum class SensorType: uint8_t
    {
        Test = 0,
        SimpleWheatherStation = 1
    };

    struct SensorData
    {
        sensorPacket::SensorPacket packet;

        uint64_t recvTime; 
    };

    struct SensorInfo
    {
        SensorType sensorType;
        uint32_t identifier;
        uint32_t softwareVersion;
        uint32_t hardwareVersion;
        uint8_t address[5];
        uint64_t initializationTime;
        uint64_t lastRecvDataTime;
    };

    struct Sensor
    {
        SensorInfo sensorInfo;

        RingBuffer<SensorData, 10> sensorData;
    };
}

#endif