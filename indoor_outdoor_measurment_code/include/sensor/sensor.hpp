#if defined(WEATHER_RECEIVER)
#pragma once
#include <stdint.h>
#include "sensor/sensor_packet.hpp"
#include "sensor/sensor_data.hpp"
#include "utilities/ring_buffer.hpp"
#include "pico/time.h"

namespace sensor
{
    constexpr int32_t maxKnownSensors = 10;
    Sensor knownSensors[maxKnownSensors];


    // Calculating CRC is for now simple sum
    int checkCrc(const uint8_t* payload, uint8_t len)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < len; i++)
            sum += *(payload++);
        
        if(sum != *payload)
            return -1;

        return 0;
    }

    int processSensorPayload(uint8_t* payload, uint8_t len)
    {
        // TODO Add checksum
        // if(checkCrc(payload, len) != 0)
            // return -1;

        uint64_t recvTime = get_absolute_time();

        // Copying is better because payload has not const len
        sensorPacket::SensorPacket recv;
        for (size_t i = 0; i < len; i++)
            recv.raw[i] = payload[i];        

        recv.General.identifierValue;
        recv.Data.identifierValue;
        recv.Info.identifierValue;

        if (recv.General.identifierValue >= maxKnownSensors)
            return -1;

        Sensor& tmpSensor = knownSensors[recv.General.identifierValue];

        switch (recv.General.header.type)
        {
        case sensorPacket::PacketType::Test:
            break;
        
        case sensorPacket::PacketType::SensorInfo:
            tmpSensor.sensorInfo.hardwareVersion = recv.Info.hardwareVersionValue;
            tmpSensor.sensorInfo.softwareVersion = recv.Info.softwareVersionValue;
            tmpSensor.sensorInfo.sensorType = static_cast<SensorType>(recv.Info.sensorType);
            tmpSensor.sensorInfo.initializationTime = recvTime;
            break;

        case sensorPacket::PacketType::SensorData:
            SensorData data {
                .packet = recv,
                .recvTime = recvTime
            };
            tmpSensor.sensorData.pushData(data);
            break;
        } 

        tmpSensor.sensorInfo.lastRecvDataTime = recvTime;

        return 0;
    }
}
#endif