#pragma once
#include <cstdint>
#include "ring_buffer.hpp"
#include "packet.hpp"
#include "sensor/sensor/sensor_config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "utilities.hpp"
#include <type_traits>
#include <utility>

namespace sensor
{
    enum class SensorType: uint32_t
    {
        Test = 0,
        SimpleWeatherSensorGeneric = 0x00010000,
        SimpleWeatherSensorBMP280  = 0x00010058,
        SimpleWeatherSensorBME280  = 0x00010060,
        SimpleWeatherSensorBME680  = 0x00010061,
        Unknown = 0xffffffff
    };

    enum class SensorReturnCode: int32_t
    {
        NotImplemented = -1,
        Ok = 0,
    };

    struct SensorInfo
    {
        uint32_t identifier = -1;
        uint32_t softwareVersion = -1;
        uint32_t hardwareVersion = -1;
        SensorType sensorType = SensorType::Unknown;

        uint8_t dataPacketsMaxCount = 0; // TODO add maximum value
        uint8_t calibrationPacketsMaxCount = 0; // TODO add maximum value

        uint8_t address[5]{0xff};
        uint64_t initializationTime = -1;
    };

    struct SensorCalibrationData
    {
        union
        {
            uint8_t raw[128];

            union
            {
                uint8_t calibRaw[26];
                struct
                {
                    uint16_t digT1;
                    int16_t digT2;
                    int16_t digT3;

                    uint16_t digP1;
                    int16_t digP2;
                    int16_t digP3;
                    int16_t digP4;
                    int16_t digP5;
                    int16_t digP6;
                    int16_t digP7;
                    int16_t digP8;
                    int16_t digP9;
                };
            } bmp280;
            
            union
            {
                uint8_t calibRaw[33];
                struct 
                {
                    uint16_t digT1;
                    int16_t digT2;
                    int16_t digT3;
                    
                    uint16_t digP1;
                    int16_t digP2;
                    int16_t digP3;
                    int16_t digP4;
                    int16_t digP5;
                    int16_t digP6;
                    int16_t digP7;
                    int16_t digP8;
                    int16_t digP9;
                    
                    uint8_t digH1;
                    int16_t digH2;
                    uint8_t digH3;
                    int16_t digH4;
                    int16_t digH5;
                    int8_t digH6;
                };
            } bme280;
            
            union
            {
                uint8_t calibRaw[64];
                struct
                {
                    
                };
            } bme680;
            
        };

        uint16_t resistorR1Value = -1;
        uint16_t resistorR2Value = -1;
    };

    struct SensorCalculatedData
    {
        uint64_t recvTime;

        union
        {
            struct 
            {
                uint32_t temperatureRaw = -1;
                uint32_t pressureRaw = -1;

                uint16_t voltageRaw = -1;
            } SimpleWeatherSensorBMP280;

            struct 
            {
                uint32_t temperatureRaw = -1;
                uint32_t pressureRaw = -1;
                uint32_t humidityRaw = -1;

                uint16_t voltageRaw = -1;
            } SimpleWeatherSensorBME280;
            
            struct 
            {
                
            } SimpleWeatherSensorBME680;
        };
        
    };

    struct SensorRawData
    {
        uint8_t dataId = -1;
        uint8_t rawData[26];
    };

    enum class SensorSingleDataType: uint8_t
    {
        Unknown,
        Temperature,
        Pressure,
        Humidity,
        Voltage
    };

    struct SensorSingleData
    {
        SensorSingleDataType dataType;
        uint64_t timestamp;
        union 
        {
            int32_t temperature;
            uint32_t pressure;
            uint16_t humidity;
            uint16_t voltage;
        };
    };

    struct SensorStatus
    {
        explicit SensorStatus(SensorInfo* sensorInfo)
        : sensorInfo(sensorInfo)
        {  
            statusReg.isSleeping = 0;
            statusReg.isDone = 0;
            statusReg.isSensorInfoKnown = 0;
            statusReg.isSensorInfoKnownSend = 0;
            statusReg.reserved = 0;
            
            statusReg.isSensorDataReceived = 0;
            statusReg.isSensorCalibReceived = 0;
            statusReg.isDataCalculated = 0;
            statusReg.isSensorCalibReqSend = 0;
            statusReg.isSensorSaved = 0;
        }
        
        struct 
        {
            uint32_t isSleeping : 1;
            /// @brief Set this flag if everything is done
            uint32_t isDone : 1;
            uint32_t isSensorInfoKnown : 1;
            uint32_t isSensorInfoKnownSend : 1;

            uint32_t isSensorDataReceived : 1;
            uint32_t isSensorCalibReceived : 1;
            
            /// @brief Set this flag when bridge send request for calib data, this is needed to avoid send multiple requests
            uint32_t isSensorCalibReqSend : 1;

            uint32_t isSensorDataReqSend : 1;

            uint32_t isDataCalculated : 1;

            uint32_t isSensorSaved : 1;
            uint32_t reserved : 22;
        } statusReg;

        uint32_t sendDataReqPackets = 0;

        /// @brief ex. 0b000101 -> this mean first and thirtd packet received
        uint32_t sendDataPackets = 0;
        
        /// @brief ex. 0b000101 -> this mean first and thirtd packet send
        uint32_t sendCalibPackets = 0; 
        
        uint32_t recvDataPackets = 0;
        uint32_t recvCalibPackets = 0; 

        uint32_t getCountOfPackets(uint32_t packetFlags)
        {
            uint32_t count = 0;
            while (packetFlags)
            {
                count++;
                packetFlags = packetFlags >> 1;
            }
            return count;
        }

        uint32_t getCountOfSendDataPackets()
        {
            return getCountOfPackets(sendDataPackets);
        }

        uint32_t getCountOfSendCalibPackets()
        {
            return getCountOfPackets(sendCalibPackets);
        }

        uint32_t getCountOfRecvDataPackets()
        {
            return getCountOfPackets(recvDataPackets);
        }

        uint32_t getCountOfRecvCalibPackets()
        {
            return getCountOfPackets(recvCalibPackets);
        }

        int32_t getNextPacketId(const uint32_t& packetFlags, const uint32_t& maxPackets)
        {
            uint32_t tmpMask = 1;

            for (int32_t i = 0; i < maxPackets; i++)
            {
                if(packetFlags & tmpMask)
                    return i;

                tmpMask <<= 1;
            }
            return -1;
        }

        /// @brief Get id of request packet with data
        /// @return If next packet not exists -1, otherwise id of packet
        int32_t getSendDataPacketId()
        {
           return getNextPacketId(sendDataPackets, sensorInfo->dataPacketsMaxCount);
        }

        /// @brief Get id of request packet with data
        /// @return If next packet not exists -1, otherwise id of packet
        int32_t getSendCalibPacketId()
        {
           return getNextPacketId(sendCalibPackets, sensorInfo->calibrationPacketsMaxCount);
        }

        int32_t getRecvDataPacketId()
        {
            return getNextPacketId(recvDataPackets, sensorInfo->dataPacketsMaxCount);
        }

        int32_t getRecvCalibPacketId()
        {
           return getNextPacketId(recvCalibPackets, sensorInfo->calibrationPacketsMaxCount);
        }
    
        void setPacketId(uint32_t& packetFlags, const uint32_t& packetId)
        {
            packetFlags |= 1 << packetId;
        }

        void setSendDataPacketId(const uint32_t& packetId)
        {
            setPacketId(sendDataPackets, packetId);
        }

        void setSendCalibPacketId(const uint32_t& packetId)
        {
            setPacketId(sendCalibPackets, packetId);
        }

        void setRecvDataPacketId(const uint32_t& packetId)
        {
            setPacketId(recvDataPackets, packetId);
        }

        void setRecvCalibPacketId(const uint32_t& packetId)
        {
            setPacketId(recvCalibPackets, packetId);
        }

        void setSendDataReqPackets(const uint32_t packetId)
        {
            setPacketId(sendDataReqPackets, packetId);
            if(getCountOfPackets(sendDataReqPackets) == sensorInfo->dataPacketsMaxCount)
            {
                statusReg.isSensorDataReqSend = 1;
            }
        }

        void resetSendDataReqPackets()
        {
            sendDataReqPackets = 0;
            statusReg.isSensorDataReqSend = 0;
        }
    private:
        SensorInfo* sensorInfo;
    };

    // template<uint32_t N = 10>
    class Sensor
    {
    public:
        Sensor()
        : status(&info)
        { }
        
        virtual int setRawData(sensorPacket::SensorPacketWithLen* packet) 
        { 
            return 0;
        }

        virtual int setCalibrationData(sensorPacket::SensorPacketWithLen* packet) 
        { 
            return 0;
        }

        virtual int getCalibrationDataStr(char* buf, size_t bufSize) const
        { 
            return snprintf(buf, bufSize, "no_calibration_data");
        }

        virtual int getResultsColsStr(char* buf, size_t bufSize) const
        {
            return snprintf(buf, bufSize, "no_results");
        }

        virtual int getResultsStr(char* buf, size_t bufSize) const
        {
            return snprintf(buf, bufSize, "no_results");
        }

        virtual int calculateData()
        { 
            return 0;
        }

        virtual SensorReturnCode getResultsJson()
        {
            return SensorReturnCode::NotImplemented;
        }

        SensorType getSensorType() const
        {
            return info.sensorType;
        }

        SensorInfo info{};
        SensorCalibrationData calibrationData{};
        SensorStatus status;

        /// @brief May be unused
        // SensorCalculatedData calculatedData{};

        RingBuffer<SensorRawData, SensorRawDataCount> lastRawDatas;
        RingBuffer<sensorPacket::SensorPacketWithLen, SensorRawDataCount> lastSensorPackets;
        
        // Received packet time
        uint64_t lastRecvDataTime = -1;
        // Send packet time
        uint64_t lastSendDataTime = -1;
        
        RingBuffer<SensorSingleData, SensorDataCount> calculatedData;

        SemaphoreHandle_t  sensorMutex = nullptr;

    };

    class SensorSimpleWeatherSensorBME280 : public Sensor
    {
    public:
        int setCalibrationData(sensorPacket::SensorPacketWithLen* packet) override
        {
            uint8_t calibPacketId = (static_cast<uint8_t>(packet->packet.General.header.type) & 0b111) - 1;
            uint8_t updateStatus = 0;

            if(calibPacketId == 0)
            {
                if(packet->size < 32)
                    return 1;
                //std::memcpy(sensorPtr->calibrationData.bmp280.calibRaw, packet.packet.CalibData.calibData, packet.size);
                calibrationData.bme280.digT1 = bytesToUint16(&packet->packet.CalibData.calibData[0]);
                calibrationData.bme280.digT2 = bytesToInt16(&packet->packet.CalibData.calibData[2]);
                calibrationData.bme280.digT3 = bytesToInt16(&packet->packet.CalibData.calibData[4]);
            
                calibrationData.bme280.digP1 = bytesToUint16(&packet->packet.CalibData.calibData[6]);
                calibrationData.bme280.digP2 = bytesToInt16(&packet->packet.CalibData.calibData[8]);
                calibrationData.bme280.digP3 = bytesToInt16(&packet->packet.CalibData.calibData[10]);
                calibrationData.bme280.digP4 = bytesToInt16(&packet->packet.CalibData.calibData[12]);
                calibrationData.bme280.digP5 = bytesToInt16(&packet->packet.CalibData.calibData[14]);
                calibrationData.bme280.digP6 = bytesToInt16(&packet->packet.CalibData.calibData[16]);
                calibrationData.bme280.digP7 = bytesToInt16(&packet->packet.CalibData.calibData[18]);
                calibrationData.bme280.digP8 = bytesToInt16(&packet->packet.CalibData.calibData[20]);
                calibrationData.bme280.digP9 = bytesToInt16(&packet->packet.CalibData.calibData[22]);

                calibrationData.bme280.digH1 = packet->packet.CalibData.calibData[25];

                updateStatus = 1;
            }
            else if(calibPacketId == 1)
            {
                if(packet->size < 16)
                    return 1;
               calibrationData.bme280.digH2 = bytesToInt16(&packet->packet.CalibData.calibData[0]);
               calibrationData.bme280.digH3 = packet->packet.CalibData.calibData[2];

                calibrationData.bme280.digH4 = packet->packet.CalibData.calibData[3];
                calibrationData.bme280.digH4 <<= 4;
                calibrationData.bme280.digH4 |= (packet->packet.CalibData.calibData[4] & 0x0f);
                
                calibrationData.bme280.digH5 = packet->packet.CalibData.calibData[5];
                calibrationData.bme280.digH5 <<= 4;
                calibrationData.bme280.digH5 |= ((packet->packet.CalibData.calibData[4] & 0xf0) >> 4);

                // sensorPtr->calibrationData.bme280.digH4 = (packet.packet.CalibData.calibData[4] & 0x0f);
                // sensorPtr->calibrationData.bme280.digH4 <<= 8;
                // sensorPtr->calibrationData.bme280.digH4 |= packet.packet.CalibData.calibData[3];
                // sensorPtr->calibrationData.bme280.digH5 = (packet.packet.CalibData.calibData[4] & 0xf0);
                // sensorPtr->calibrationData.bme280.digH5 <<= 4;
                // sensorPtr->calibrationData.bme280.digH5 |= packet.packet.CalibData.calibData[5];

                calibrationData.bme280.digH6 = packet->packet.CalibData.calibData[6];
                
                calibrationData.resistorR1Value = bytesToInt16(&packet->packet.CalibData.calibData[7]);
                calibrationData.resistorR2Value = bytesToInt16(&packet->packet.CalibData.calibData[9]);

                updateStatus = 1;
            }
            else
            {
                // log("Sensor", LogLevel::Warning, "CalibPacketId does not match!");
                return 2;
            }

            return 0;
        }

        int getCalibrationDataStr(char* buf, size_t bufSize) const override
        {
            return snprintf(buf, bufSize, 
                "bme280.digT1=%u,bme280.digT2=%d,bme280.digT3=%d,bme280.digP1=%u,bme280.digP2=%d,bme280.digP3=%d,bme280.digP4=%d,bme280.digP5=%d,bme280.digP6=%d,bme280.digH1=%d,resistorR1Value=%u,resistorR2Value=%u",
                calibrationData.bme280.digT1,
                calibrationData.bme280.digT2,
                calibrationData.bme280.digT3,
                calibrationData.bme280.digP1,
                calibrationData.bme280.digP2,
                calibrationData.bme280.digP3,
                calibrationData.bme280.digP4,
                calibrationData.bme280.digP5,
                calibrationData.bme280.digP6,
                calibrationData.bme280.digH1,
                calibrationData.resistorR1Value,
                calibrationData.resistorR2Value
            );
        }

        int getResultsColsStr(char* buf, size_t bufSize) const override
        {
            return snprintf(buf, bufSize, "temperature,pressure,humidity,voltage");
        }

        int getResultsStr(char* buf, size_t bufSize) const override
        {
            return snprintf(buf, bufSize, 
                "%lu,%lu,%lu,%u",
                calculatedData.temperature,
                calculatedData.pressure,
                calculatedData.humidity,
                calculatedData.voltage
            );
        }

        int calculateData() override
        {
            /// @todo check if calibration data, sensor info and raw data is known

            return 0;
        }

        struct CalculatedData 
        {
            uint32_t temperature = -1;
            uint32_t pressure = -1;
            uint32_t humidity = -1;

            uint16_t voltage = -1;
        };


    private:
        
        struct
        {
            struct 
            {
                uint16_t digT1;
                int16_t digT2;
                int16_t digT3;
                
                uint16_t digP1;
                int16_t digP2;
                int16_t digP3;
                int16_t digP4;
                int16_t digP5;
                int16_t digP6;
                int16_t digP7;
                int16_t digP8;
                int16_t digP9;
                
                uint8_t digH1;
                int16_t digH2;
                uint8_t digH3;
                int16_t digH4;
                int16_t digH5;
                int8_t digH6;
            } bme280;
            uint16_t resistorR1Value = -1;
            uint16_t resistorR2Value = -1;
        } calibrationData;

        CalculatedData calculatedData;
    };

}