#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "packet.hpp"

enum class PacketToProcessType: int32_t
{
    Unknown,
    RecvPacket,
    SendPacket
};

struct PacketToProcess
{
    PacketToProcessType packetType;
    union
    {
        struct 
        {
            sensorPacket::SensorPacketWithLen radioPacket;
            uint64_t recvTime;
        } recvPacket;
        struct
        {
            sensorPacket::SensorPacketWithLen radioPacket;
            
        } sendPacket;
        
    };
};

struct PacketProcessorTaskArg
{
    QueueHandle_t packetToProcessQueue = nullptr;
    QueueHandle_t toRadioTaskQueue = nullptr;
    QueueHandle_t fileStorageQueue = nullptr;
};

void packetProcessorTask(void* arg);
