#if defined(WEATHER_RECEIVER)
#pragma once
#include <stdint.h>
#include "sensor/sensor_packet.hpp"
#include "sensor/sensor_data.hpp"
#include "utilities/ring_buffer.hpp"
#include "utilities/list.hpp"
#include "utilities.hpp"

namespace sensor
{
    constexpr int32_t maxKnownSensors = 10;
    List<Sensor, maxKnownSensors> knownSensors;

    int processSensorPayload(uint8_t* payload, uint8_t len)
    {
        if(sensorPacket::checkCrc(payload, len) != 0)
            return -1;

        uint64_t recvTime = getTime();

        // Copying is better because payload has not const len
        sensorPacket::SensorPacket recv;
        for (size_t i = 0; i < len; i++)
            recv.raw[i] = payload[i];

        recv.General.identifierValue; // Temporary, used to debug

        Sensor tmpSensor;
        Sensor* ptrSensor = &tmpSensor;

        // Looking for known identifier
        int isKnownSensor = knownSensors.valueByExpression([](Sensor& refSensor, void* args)->int {
            sensorPacket::SensorPacket* tmp = reinterpret_cast<sensorPacket::SensorPacket*>(args);
            if(refSensor.sensorInfo.identifier == tmp->General.identifierValue)
                return 0;
            return -1;
        }, ptrSensor, &recv);

        if(isKnownSensor == -1)
        {
            tmpSensor.sensorInfo.identifier = recv.General.identifierValue;
            if(knownSensors.add(tmpSensor) == -1)
            {
                // TODO if list is full
            }

            knownSensors.valueAt(0, ptrSensor);
        }

        switch (recv.General.header.type)
        {
        case sensorPacket::PacketType::Test:
            break;
        
        case sensorPacket::PacketType::SensorInfo:
            ptrSensor->sensorInfo.hardwareVersion = recv.Info.hardwareVersionValue;
            ptrSensor->sensorInfo.softwareVersion = recv.Info.softwareVersionValue;
            ptrSensor->sensorInfo.sensorType = static_cast<SensorType>(recv.Info.sensorType);
            ptrSensor->sensorInfo.initializationTime = recvTime;
            break;

        case sensorPacket::PacketType::SensorData:
            SensorData data {
                .packet = recv,
                .recvTime = recvTime
            };
            ptrSensor->sensorData.pushData(data);
            break;
        } 

        ptrSensor->sensorInfo.lastRecvDataTime = recvTime;

        return 0;
    }
}
#endif