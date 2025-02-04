#pragma once
#include <stdint.h>

namespace sensorPacket
{
    enum PacketDirection
    {
        Request = 0b00,
        Response = 0b01,
        Reserved0 = 0b10,
        Reserved1 = 0b11,
    };

    enum PacketType
    {
        Test = 0b00000,
        SensorData = 0b00001,
        SensorInfo = 0b00010
    };

    #pragma pack(push, 1)
    struct Header
    {
        PacketDirection direction : 2;
        uint8_t errorFlag : 1;
        PacketType type : 5;
    };
    #pragma pack(pop)

    union SensorPacket
    {
        uint8_t raw[32]{0};

        struct 
        {
            Header header;
            uint8_t crc;
            union
            {
                uint8_t identifierRaw[4];
                uint32_t identifierValue;
            };
            union
            {
                uint8_t rawData[30];
            };
        } General;

        struct
        {
            Header header;
            uint8_t crc;
            union
            {
                uint8_t identifierRaw[4];
                uint32_t identifierValue;
            };
            union
            {
                uint8_t temperatureRaw[4];
                uint32_t temperatureValue;
            };
            union
            {
                uint8_t pressureRaw[4];
                uint32_t pressureValue;
            };
            union
            {
                uint8_t humidityRaw[4];
                uint32_t humidityValue;
            };
            union
            {
                uint8_t batteryVoltageRaw[4];
                uint32_t batteryVoltageValue;
            };
        } Data;

        struct 
        {
            Header header;
            uint8_t crc;
            union
            {
                uint8_t identifierRaw[4];
                uint32_t identifierValue;
            };
            union 
            {
                uint8_t sensorType;
            };
            union
            {
                uint8_t softwareVersionRaw[4];
                uint32_t softwareVersionValue;
            };
            union
            {
                uint8_t hardwareVersionRaw[4];
                uint32_t hardwareVersionValue;
            };
        } Info;
    };
}

