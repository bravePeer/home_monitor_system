#pragma once
#if __has_include(<cstdint>)
    #include <cstdint>
#else
    #include <stdint.h>
#endif

namespace sensorPacket
{
    /// Packet structure: <Header> <CRC> <Data up to 62 bytes>
    ///
    /// Header: TTTT TEDD
    ///     T -> Packet type
    ///     E -> Packet error flag
    ///     D -> Packet direction
    ///
    /// CRC is sum of Header and Data bytes
    ///

    /// @brief Possible directions of packet
    /// 
    ///  - Request (0b00) Receiver sends data (request packet) to Sensor
    ///
    ///  - Response (0b10) Sensor answers to Receiver Request 
    ///
    ///  - Report (0b11) Sensor sends data to Receiver
    ///
    enum class PacketDirection : uint8_t
    {
        Request  = 0b00,
        Reserved = 0b01,
        Response = 0b10,
        Report   = 0b11,
    };

    enum class PacketType : uint8_t
    {
        Test             = 0b00000,
        SensorInfo       = 0b00001,
        SensorCalibData0 = 0b00010,
        SensorCalibData1 = 0b00011,
        
        SensorData0      = 0b10000,
        SensorData1      = 0b10001,
    };

    enum class PacketError : uint8_t
    {
        NoError = 0,
        Error   = 1
    };

    #pragma pack(push, 1)
    /// Header: TTTT TEDD
    ///     T -> Packet type
    ///     E -> Packet error flag
    ///     D -> Packet direction
    struct Header
    {
        // uint8_t headerRaw;
        PacketDirection direction : 2;
        PacketError errorFlag : 1;
        PacketType type : 5;
    };
    #pragma pack(pop)

    // constexpr uint8_t PacketTypeMask = 0xf8;
    // constexpr uint8_t PacketErrorMask = 0x04;
    // constexpr uint8_t PacketDirectionMask = 0x03;

    // inline Header createHeader(PacketDirection dir, PacketError err, PacketType typ)
    // {
    //     Header header;
    //     header.headerRaw = (((static_cast<uint8_t>(typ) << 1) | static_cast<uint8_t>(err)) << 2) | static_cast<uint8_t>(dir);
    // }
    
    
    #pragma pack(push, 1)
    /// Packet structure: <Header> <CRC> <Data up to 62 bytes>
    ///
    /// Header: TTTT TEDD
    ///     T -> Packet type
    ///     E -> Packet error flag
    ///     D -> Packet direction
    ///
    /// CRC is sum of Header and Data bytes
    ///
    union SensorPacket
    {
        uint8_t raw[32];

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
                uint8_t rawData[26];
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
        } Data; // 22 bytes

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
        } Info; // 15 bytes

        struct
        {
            Header header;
            uint8_t crc;
            union
            {
                uint8_t identifierRaw[4];
                uint32_t identifierValue;
            };
            uint8_t calibData[26]; 
        } CalibData; // 32 bytes
    };
    #pragma pack(pop)

    struct SensorPacketWithLen
    {
        SensorPacket packet;
        uint8_t dataLen = 0;
    };

    inline int8_t generateCrc(uint8_t* payload, uint8_t len)
    {
        if(len < 3)
            return -1;

        uint8_t sum = payload[0];
        for (uint8_t i = 2; i < len; i++)
            sum += payload[i];
        
        payload[1] = sum;
        return 0;
    }

    // Calculating CRC is for now simple sum
    inline int8_t checkCrc(const uint8_t* payload, uint8_t len)
    {
        if(len < 2)
            return -1;
        
        uint8_t sum = payload[0];
        for (uint8_t i = 2; i < len; i++)
            sum += payload[i];
        
        if(sum != payload[1])
            return -1;

        return 0;
    }
}
