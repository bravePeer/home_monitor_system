#pragma once
#include <cstring>
#include <cstdio>
#include <string>
#include "packet.hpp"

namespace sensorPacket
{
    struct PacketStr
    {
        char buf[128]{0};
    };

    inline const char* packetHeaderTypeToStr(PacketType typ)
    {
        switch (typ)
        {
        case PacketType::Test:
            return "Test";
        case PacketType::SensorInfo:
            return "SensorInfo";
        case PacketType::SensorCalibData0:
            return "SensorCalibData0";
        case PacketType::SensorCalibData1:
            return "SensorCalibData1";
        case PacketType::SensorData0:
            return "SensorData0";
        case PacketType::SensorData1:
            return "SensorData1";
        }
        return "Unknown";
    }

    inline const char* packetHeaderErrorToStr(PacketError err)
    {
        if(err == PacketError::Error)
            return " Error ";
        return "NoError";
    }

    inline const char* packetHeaderDirectionToStr(PacketDirection dir)
    {
        switch (dir)
        {
        case PacketDirection::Request:
            return "Request ";
        case PacketDirection::Reserved:
            return "Reserved";
        case PacketDirection::Response:
            return "Response";
        case PacketDirection::Report:
            return " Report ";
        }
        return "Unknown";
    }

    inline PacketStr packetHeaderToStr(const sensorPacket::Header* header)
    {
        PacketStr str;
        std::sprintf(str.buf, "%s | %s | %s", 
            packetHeaderDirectionToStr(header->direction),
            packetHeaderErrorToStr(header->errorFlag),
            packetHeaderTypeToStr(header->type)    
        );

        return str;
    }

    inline void bytesToHexStr(const uint8_t* bytes, uint8_t bytesLen, char* str)
    {
        for (uint8_t i = 0; i < bytesLen; i++)
        {
            str[i * 2] = (bytes[i] & 0xf0) >> 4;
            str[i * 2 + 1] = bytes[i] & 0x0f;
        }

        for (uint8_t i = 0; i < bytesLen * 2; i++)
        {
            if(str[i] <= 9)
                str[i] += '0';
            else
                str[i] += 'A' - 10;
        }
    }

    inline PacketStr packetToStrInfo(const SensorPacket* packet, uint8_t len)
    {
        PacketStr str;
        char headerBytes[3]{0};
        bytesToHexStr(packet->raw, 1, headerBytes);
        char crcBytes[3]{0};
        bytesToHexStr(packet->raw + 1, 1, crcBytes);
        char dataBytes[61]{0};
        bytesToHexStr(packet->raw + 2, len - 2, dataBytes);
        std::sprintf(str.buf, "Packet with %d bytes, Header:  %s, CRC: %s, Data: %s", len, headerBytes, crcBytes, dataBytes);

        return str;
    }

    inline PacketStr packetToStr(const sensorPacket::SensorPacket* packet, uint8_t len)
    {
        PacketStr str;

        bytesToHexStr(packet->raw, len, str.buf);
        
        return str;
    }

    inline PacketStr packetToStr(const uint8_t* data, uint8_t len)
    {
        PacketStr str;

        bytesToHexStr(data, len, str.buf);
        
        return str;
    }
}
