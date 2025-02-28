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
        SimpleWheatherStation = 1,
        Unknown = 0xff
    };

    struct SensorData
    {
        sensorPacket::SensorPacket packet;

        uint64_t recvTime; 
    };

    struct SensorInfo
    {
        SensorType sensorType = SensorType::Unknown;
        uint32_t identifier = -1;
        uint32_t softwareVersion = -1;
        uint32_t hardwareVersion = -1;
        uint8_t address[5]{0xff};
        uint64_t initializationTime = -1;
        uint64_t lastRecvDataTime = -1;
    };

    struct Sensor
    {
        SensorInfo sensorInfo{};

        RingBuffer<SensorData, 10> sensorData;
    };
}

#endif