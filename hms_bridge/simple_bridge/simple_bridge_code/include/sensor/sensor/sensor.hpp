#pragma once
#include <cstdint>
#include <cstring>
#include "sensor/sensor/sensor_common.hpp"
#include "utilities.hpp"
#include "list.hpp"
#include "packet.hpp"
#include "general/platform_time.hpp"
#include "receiver/logger/logger.hpp"
#include "sensor/sensor/sensor_config.hpp"
#include "sensor/sensor/sensor_calculation.hpp"
#include "sensor/sensor/sensor_utils.hpp"

namespace sensor
{
    /// @brief List of known sensors
    /// @todo Make it thread safe and not global
    inline List<Sensor, MaxKnownSensors> knownSensors;

    inline RingBuffer<Sensor*, MaxSensorsToSendData> sensorsWantToSendData;

    static int processRecvData(const sensorPacket::SensorPacketWithLen& packet, Sensor* sensorPtr)
    {
        if(sensorPtr->status.statusReg.isSensorInfoKnown)
        {
            if(sensorPtr->status.getCountOfRecvDataPackets() == sensorPtr->info.dataPacketsMaxCount)
            {
                sensorPtr->status.recvDataPackets = 0;
                sensorPtr->status.sendDataPackets = 0;
                sensorPtr->status.statusReg.isDataCalculated = 0;
            }
        }

        log("Sensor", LogLevel::Info, "Processing data packet...");
        uint8_t dataPacketId = (static_cast<uint8_t>(packet.packet.General.header.type) & 0b111);
        
        sensorPtr->lastSensorPackets.push(packet);
        sensor::SensorRawData data;
        memcpy(data.rawData, &packet.packet.raw[6], packet.size - 6);
        data.dataId = dataPacketId;

        sensorPtr->lastRawDatas.push(data);

        sensorPtr->status.recvDataPackets |= 1 << dataPacketId;
        
        if(sensorPtr->status.statusReg.isSensorInfoKnown)
        {
            // Waits for all data packets then sets flag
            // [ ] Set recv data packets flag - not tested
            if(sensorPtr->status.getCountOfRecvDataPackets() == sensorPtr->info.dataPacketsMaxCount)
            {
                sensorPtr->status.statusReg.isSensorDataReceived = 1;
                log("Sensor", LogLevel::Info, "Set flag: isSensorDataReceived");
            }
        }

        return 0;
    }

    /// @brief 
    /// @param packet 
    /// @param sensorPtr 
    /// @return If 0 success, -1 -> undefined error, 1 -> packet bad size, 2 -> packet calibration id not match
    static int processRecvCalib(const sensorPacket::SensorPacketWithLen& packet, Sensor* sensorPtr)
    {
        uint8_t calibPacketId = (static_cast<uint8_t>(packet.packet.General.header.type) & 0b111) - 1;
        uint8_t updateStatus = 0;

        switch (sensorPtr->info.sensorType)
        {
        case sensor::SensorType::Test:
            if(calibPacketId >= sensorPtr->info.calibrationPacketsMaxCount)
                return 2;
            std::memcpy(sensorPtr->calibrationData.raw, packet.packet.CalibData.calibData, packet.size - 2);
            break;
        case sensor::SensorType::SimpleWeatherSensorBMP280:
            if(calibPacketId == 0)
            {
                if(packet.size < 30)
                    return 1;
                //std::memcpy(sensorPtr->calibrationData.bmp280.calibRaw, packet.packet.CalibData.calibData, packet.size);
                sensorPtr->calibrationData.bmp280.digT1 = bytesToUint16(&packet.packet.CalibData.calibData[0]);
                sensorPtr->calibrationData.bmp280.digT2 = bytesToInt16(&packet.packet.CalibData.calibData[2]);
                sensorPtr->calibrationData.bmp280.digT3 = bytesToInt16(&packet.packet.CalibData.calibData[4]);
                sensorPtr->calibrationData.bmp280.digP1 = bytesToUint16(&packet.packet.CalibData.calibData[6]);
                sensorPtr->calibrationData.bmp280.digP2 = bytesToInt16(&packet.packet.CalibData.calibData[8]);
                sensorPtr->calibrationData.bmp280.digP3 = bytesToInt16(&packet.packet.CalibData.calibData[10]);
                sensorPtr->calibrationData.bmp280.digP4 = bytesToInt16(&packet.packet.CalibData.calibData[12]);
                sensorPtr->calibrationData.bmp280.digP5 = bytesToInt16(&packet.packet.CalibData.calibData[14]);
                sensorPtr->calibrationData.bmp280.digP6 = bytesToInt16(&packet.packet.CalibData.calibData[16]);
                sensorPtr->calibrationData.bmp280.digP7 = bytesToInt16(&packet.packet.CalibData.calibData[18]);
                sensorPtr->calibrationData.bmp280.digP8 = bytesToInt16(&packet.packet.CalibData.calibData[20]);
                sensorPtr->calibrationData.bmp280.digP9 = bytesToInt16(&packet.packet.CalibData.calibData[22]);

                updateStatus = 1;
            }
            else if(calibPacketId == 1)
            {
                if(packet.size < 10)
                    return 1;
                sensorPtr->calibrationData.resistorR1Value = bytesToInt16(&packet.packet.CalibData.calibData[0]);
                sensorPtr->calibrationData.resistorR2Value = bytesToInt16(&packet.packet.CalibData.calibData[2]);

                updateStatus = 1;
            }
            else
            {
                log("Sensor", LogLevel::Warning, "CalibPacketId does not match!");
                return 2;
            }
            break;
        case sensor::SensorType::SimpleWeatherSensorBME280:
            if(calibPacketId == 0)
            {
                if(packet.size < 32)
                    return 1;
                //std::memcpy(sensorPtr->calibrationData.bmp280.calibRaw, packet.packet.CalibData.calibData, packet.size);
                sensorPtr->calibrationData.bme280.digT1 = bytesToUint16(&packet.packet.CalibData.calibData[0]);
                sensorPtr->calibrationData.bme280.digT2 = bytesToInt16(&packet.packet.CalibData.calibData[2]);
                sensorPtr->calibrationData.bme280.digT3 = bytesToInt16(&packet.packet.CalibData.calibData[4]);
                
                sensorPtr->calibrationData.bme280.digP1 = bytesToUint16(&packet.packet.CalibData.calibData[6]);
                sensorPtr->calibrationData.bme280.digP2 = bytesToInt16(&packet.packet.CalibData.calibData[8]);
                sensorPtr->calibrationData.bme280.digP3 = bytesToInt16(&packet.packet.CalibData.calibData[10]);
                sensorPtr->calibrationData.bme280.digP4 = bytesToInt16(&packet.packet.CalibData.calibData[12]);
                sensorPtr->calibrationData.bme280.digP5 = bytesToInt16(&packet.packet.CalibData.calibData[14]);
                sensorPtr->calibrationData.bme280.digP6 = bytesToInt16(&packet.packet.CalibData.calibData[16]);
                sensorPtr->calibrationData.bme280.digP7 = bytesToInt16(&packet.packet.CalibData.calibData[18]);
                sensorPtr->calibrationData.bme280.digP8 = bytesToInt16(&packet.packet.CalibData.calibData[20]);
                sensorPtr->calibrationData.bme280.digP9 = bytesToInt16(&packet.packet.CalibData.calibData[22]);

                sensorPtr->calibrationData.bme280.digH1 = packet.packet.CalibData.calibData[25];

                updateStatus = 1;
            }
            else if(calibPacketId == 1)
            {
                if(packet.size < 16)
                    return 1;
                sensorPtr->calibrationData.bme280.digH2 = bytesToInt16(&packet.packet.CalibData.calibData[0]);
                sensorPtr->calibrationData.bme280.digH3 = packet.packet.CalibData.calibData[2];

                sensorPtr->calibrationData.bme280.digH4 = packet.packet.CalibData.calibData[3];
                sensorPtr->calibrationData.bme280.digH4 <<= 4;
                sensorPtr->calibrationData.bme280.digH4 |= (packet.packet.CalibData.calibData[4] & 0x0f);
                
                sensorPtr->calibrationData.bme280.digH5 = packet.packet.CalibData.calibData[5];
                sensorPtr->calibrationData.bme280.digH5 <<= 4;
                sensorPtr->calibrationData.bme280.digH5 |= ((packet.packet.CalibData.calibData[4] & 0xf0) >> 4);

                // sensorPtr->calibrationData.bme280.digH4 = (packet.packet.CalibData.calibData[4] & 0x0f);
                // sensorPtr->calibrationData.bme280.digH4 <<= 8;
                // sensorPtr->calibrationData.bme280.digH4 |= packet.packet.CalibData.calibData[3];
                
                // sensorPtr->calibrationData.bme280.digH5 = (packet.packet.CalibData.calibData[4] & 0xf0);
                // sensorPtr->calibrationData.bme280.digH5 <<= 4;
                // sensorPtr->calibrationData.bme280.digH5 |= packet.packet.CalibData.calibData[5];

                sensorPtr->calibrationData.bme280.digH6 = packet.packet.CalibData.calibData[6];
                
                sensorPtr->calibrationData.resistorR1Value = bytesToInt16(&packet.packet.CalibData.calibData[7]);
                sensorPtr->calibrationData.resistorR2Value = bytesToInt16(&packet.packet.CalibData.calibData[9]);

                updateStatus = 1;
            }
            else
            {
                log("Sensor", LogLevel::Warning, "CalibPacketId does not match!");
                return 2;
            }
            break;
        case sensor::SensorType::SimpleWeatherSensorBME680:
            log("Sensor", LogLevel::Warning, "Not implemented SensorType: 0x%lx", static_cast<uint32_t>(sensorPtr->info.sensorType));
            return -1;
        default:
            log("Sensor", LogLevel::Warning, "Unknown SensorType: 0x%lx", static_cast<uint32_t>(sensorPtr->info.sensorType));
            return -1;
        }

        if(updateStatus)
        {
            sensorPtr->status.recvCalibPackets |= 1 << calibPacketId;

            // [ ] Set recv calibration packets flag - not tested
            if(sensorPtr->status.getCountOfRecvCalibPackets() == sensorPtr->info.calibrationPacketsMaxCount)
                sensorPtr->status.statusReg.isSensorCalibReceived = 1;
        }

        return 0;
    }


    inline int isSensorKnown(uint32_t identifier, sensor::Sensor** ptrSensor, List<Sensor, MaxKnownSensors>* knownSensorsLocal)
    {
        int isKnownSensor = knownSensorsLocal->valueByExpression([](Sensor& refSensor, void* args)->int {
            if(refSensor.info.identifier == *static_cast<uint32_t*>(args))
                return 0;
            return -1;
        }, *ptrSensor, &identifier);

        return isKnownSensor;
    }

    /// @brief 
    /// @return On success return 0 else -1 
    inline int addSensor(uint32_t identifier, sensor::Sensor** ptrSensor, List<Sensor, MaxKnownSensors>* knownSensorsLocal)
    {
        log("Sensor", LogLevel::Info, "Sensor not known, adding sensor, id: 0x%lx", identifier);

        if(knownSensorsLocal->addEmpty() == -1)
        {
            /// @todo if list is full
            return -1;
        }

        knownSensorsLocal->valueAt(0, *ptrSensor);
        (*ptrSensor)->info.identifier = identifier;

        return 0;
    }


    inline int processRecvPacket(sensorPacket::SensorPacketWithLen& packet, sensor::Sensor* ptrSensor, List<Sensor, MaxKnownSensors>* knownSensorsLocal)
    {
        int retVal = 0;

        switch (packet.packet.General.header.type)
        {
        case sensorPacket::PacketType::SensorData0:
        case sensorPacket::PacketType::SensorData1:
        case sensorPacket::PacketType::SensorData2:
        case sensorPacket::PacketType::SensorData3:
        case sensorPacket::PacketType::SensorData4:
        case sensorPacket::PacketType::SensorData5:
        case sensorPacket::PacketType::SensorData6:
        case sensorPacket::PacketType::SensorData7:
            retVal = processRecvData(packet, ptrSensor);
            break;
        
        case sensorPacket::PacketType::SensorInfo:
            log("Sensor", LogLevel::Info, "Processing info packet...");
            ptrSensor->info.hardwareVersion = packet.packet.Info.hardwareVersionValue;
            ptrSensor->info.softwareVersion = packet.packet.Info.softwareVersionValue;
            ptrSensor->info.sensorType = static_cast<SensorType>(packet.packet.Info.sensorType);
            ptrSensor->info.dataPacketsMaxCount = packet.packet.Info.dataPacketsMaxCount;
            ptrSensor->info.calibrationPacketsMaxCount = packet.packet.Info.calibrationPacketsMaxCount;

            ptrSensor->info.initializationTime = getTime();
            ptrSensor->status.statusReg.isSensorInfoKnown = 1;
            
            // Waits for all data packets then sets flag
            // [ ] Set recv data packets flag - not tested
            if(ptrSensor->status.getCountOfRecvDataPackets() == ptrSensor->info.dataPacketsMaxCount)
            {
                ptrSensor->status.statusReg.isSensorDataReceived = 1;
            }

            break;

        case sensorPacket::PacketType::SensorCalibData0:
        case sensorPacket::PacketType::SensorCalibData1:
        case sensorPacket::PacketType::SensorCalibData2:
        case sensorPacket::PacketType::SensorCalibData3:
        case sensorPacket::PacketType::SensorCalibData4:
        case sensorPacket::PacketType::SensorCalibData5:
        case sensorPacket::PacketType::SensorCalibData6:
            retVal = processRecvCalib(packet, ptrSensor);
            break;
        default:
            log("Sensor", LogLevel::Warning, "Undefined packet");
            break;
        }

        return retVal;
    }



    inline int processRecvPayload(sensorPacket::SensorPacketWithLen& packet)
    {
        uint64_t recvTime = getTime();

        // Copying is better because payload has not const len
        // sensorPacket::SensorPacket recv;
        // for (size_t i = 0; i < len; i++)
        //     recv.raw[i] = payload[i];

        Sensor tmpSensor;
        Sensor* ptrSensor = &tmpSensor;

        // Looking for known identifier
        int isKnownSensor = knownSensors.valueByExpression([](Sensor& refSensor, void* args)->int {
            sensorPacket::SensorPacket* tmp = static_cast<sensorPacket::SensorPacket*>(args);
            if(refSensor.info.identifier == tmp->General.identifierValue)
                return 0;
            return -1;
        }, ptrSensor, &packet.packet);

        if(isKnownSensor == -1)
        {
            tmpSensor.info.identifier = packet.packet.General.identifierValue;
            log("Sensor", LogLevel::Info, "Sensor not known, adding sensor, id: 0x%lx", ptrSensor->info.identifier);

            if(knownSensors.add(tmpSensor) == -1)
            {
                // TODO if list is full
            }

            knownSensors.valueAt(0, ptrSensor);
            // memset(&ptrSensor->status.statusReg, 0, 4);
            // sensorsWantToSendData.push(ptrSensor);
        }

        int retVal = 0;

        switch (packet.packet.General.header.type)
        {
        case sensorPacket::PacketType::SensorData0:
        case sensorPacket::PacketType::SensorData1:
        case sensorPacket::PacketType::SensorData2:
        case sensorPacket::PacketType::SensorData3:
        case sensorPacket::PacketType::SensorData4:
        case sensorPacket::PacketType::SensorData5:
        case sensorPacket::PacketType::SensorData6:
        case sensorPacket::PacketType::SensorData7:
            retVal = processRecvData(packet, ptrSensor);
            break;
        
        case sensorPacket::PacketType::SensorInfo:
            log("Sensor", LogLevel::Info, "Processing info packet...");
            ptrSensor->info.hardwareVersion = packet.packet.Info.hardwareVersionValue;
            ptrSensor->info.softwareVersion = packet.packet.Info.softwareVersionValue;
            ptrSensor->info.sensorType = static_cast<SensorType>(packet.packet.Info.sensorType);
            ptrSensor->info.dataPacketsMaxCount = packet.packet.Info.dataPacketsMaxCount;
            ptrSensor->info.calibrationPacketsMaxCount = packet.packet.Info.calibrationPacketsMaxCount;

            ptrSensor->info.initializationTime = recvTime;
            ptrSensor->status.statusReg.isSensorInfoKnown = 1;
            
            // Waits for all data packets then sets flag
            // [ ] Set recv data packets flag - not tested
            if(ptrSensor->status.getCountOfRecvDataPackets() == ptrSensor->info.dataPacketsMaxCount)
            {
                ptrSensor->status.statusReg.isSensorDataReceived = 1;
            }

            break;

        case sensorPacket::PacketType::SensorCalibData0:
        case sensorPacket::PacketType::SensorCalibData1:
        case sensorPacket::PacketType::SensorCalibData2:
        case sensorPacket::PacketType::SensorCalibData3:
        case sensorPacket::PacketType::SensorCalibData4:
        case sensorPacket::PacketType::SensorCalibData5:
        case sensorPacket::PacketType::SensorCalibData6:
            retVal = processRecvCalib(packet, ptrSensor);
            break;
        default:
            log("Sensor", LogLevel::Warning, "Undefined packet");
            break;
        }

        ptrSensor->lastRecvDataTime = recvTime;

        return retVal;
    }

    inline int processSendPayload(sensorPacket::SensorPacketWithLen& packet)
    {
        uint64_t sendTime = getTime();

        Sensor tmpSensor;
        Sensor* ptrSensor = &tmpSensor;

        // Looking for known identifier
        int isKnownSensor = knownSensors.valueByExpression([](Sensor& refSensor, void* args)->int {
            sensorPacket::SensorPacket* tmp = reinterpret_cast<sensorPacket::SensorPacket*>(args);
            if(refSensor.info.identifier == tmp->General.identifierValue)
                return 0;
            return -1;
        }, ptrSensor, &packet.packet);

        if(isKnownSensor == -1)
        {
           log("Sensor", LogLevel::Error, "Sensor not known 0x%lx", ptrSensor->info.identifier);
           return -1;
        }

        switch (packet.packet.General.header.type)
        {
        case sensorPacket::PacketType::SensorData0:
        case sensorPacket::PacketType::SensorData1:
        case sensorPacket::PacketType::SensorData2:
        case sensorPacket::PacketType::SensorData3:
        case sensorPacket::PacketType::SensorData4:
        case sensorPacket::PacketType::SensorData5:
        case sensorPacket::PacketType::SensorData6:
        case sensorPacket::PacketType::SensorData7:
        {
            uint8_t tmp = static_cast<uint8_t>(packet.packet.General.header.type);
            tmp &= 0x03;
            ptrSensor->status.sendDataPackets |= 1 << tmp;
            break;
        }
        case sensorPacket::PacketType::SensorInfo:
            break;

        case sensorPacket::PacketType::SensorCalibData0:
        case sensorPacket::PacketType::SensorCalibData1:
        case sensorPacket::PacketType::SensorCalibData2:
        case sensorPacket::PacketType::SensorCalibData3:
        case sensorPacket::PacketType::SensorCalibData4:
        case sensorPacket::PacketType::SensorCalibData5:
        case sensorPacket::PacketType::SensorCalibData6:
        {
            uint8_t tmp = static_cast<uint8_t>(packet.packet.General.header.type);
            tmp &= 0x03;
            tmp--;
            ptrSensor->status.sendCalibPackets |= 1 << tmp;
            
            break;
        }
        default:
            log("Sensor", LogLevel::Warning, "Undefined packet");
            break;
        } 

        return 0;
    }

    inline RingBuffer<sensorPacket::SensorPacketWithLen, 10> prepareSendPayload(Sensor* sensorPtr)
    {
        RingBuffer<sensorPacket::SensorPacketWithLen, 10> packets;
        
        if(!sensorPtr->status.statusReg.isSensorInfoKnown)
        {
            sensorPacket::SensorPacketWithLen packet;
            packet.packet.General.header.direction = sensorPacket::PacketDirection::Request;
            packet.packet.General.header.errorFlag = sensorPacket::PacketError::NoError;
            packet.packet.General.header.type = sensorPacket::PacketType::SensorInfo;
            packet.packet.General.identifierValue = sensorPtr->info.identifier;
            packet.size = 6;
            packets.push(packet);
            return packets;
        }

        for (uint8_t i = 0; i < sensorPtr->info.dataPacketsMaxCount; i++)
        {
            // If not received packet and not mark as send
            if(!(sensorPtr->status.recvDataPackets & (1 << i)) && !(sensorPtr->status.sendDataPackets & (1 << i)))
            {
                log("Sensor", LogLevel::Debug, "in if 0x%lx", sensorPtr->status.sendDataPackets);

                // Set as send
                sensorPtr->status.sendDataPackets |= 1 << i;

                sensorPacket::SensorPacketWithLen packet;
                packet.packet.General.header.direction = sensorPacket::PacketDirection::Request;
                packet.packet.General.header.errorFlag = sensorPacket::PacketError::NoError;
                uint8_t tmp = static_cast<uint8_t>(sensorPacket::PacketType::SensorData0);
                tmp |= i;
                packet.packet.General.header.type = static_cast<sensorPacket::PacketType>(tmp);
                packet.packet.General.identifierValue = sensorPtr->info.identifier;
                packet.size = 6;
                packets.push(packet);
            }
        }
        
        for (uint8_t i = 0; i < sensorPtr->info.calibrationPacketsMaxCount; i++)
        {
            // If not received packet and not mark as send
            if(!(sensorPtr->status.recvCalibPackets & (1 << i)) && !(sensorPtr->status.sendCalibPackets & (1 << i)))
            {
                // Set as send
                sensorPtr->status.sendCalibPackets |= 1 << i;

                sensorPacket::SensorPacketWithLen packet;
                packet.packet.General.header.direction = sensorPacket::PacketDirection::Request;
                packet.packet.General.header.errorFlag = sensorPacket::PacketError::NoError;
                uint8_t tmp = static_cast<uint8_t>(sensorPacket::PacketType::SensorCalibData0) - 1;
                tmp |= i;
                tmp++;
                packet.packet.General.header.type = static_cast<sensorPacket::PacketType>(tmp);
                packet.packet.General.identifierValue = sensorPtr->info.identifier;
                packet.size = 6;
                packets.push(packet);
            }
        }
        
        return packets;
    }

    /// @brief Prepare send payload for one sensor
    /// @return 
    inline RingBuffer<sensorPacket::SensorPacketWithLen, 10> prepareSendPayload()
    {
        RingBuffer<sensorPacket::SensorPacketWithLen, 10> allPackets;
        Sensor* sensorPtr = nullptr;

        if(sensorsWantToSendData.pop(sensorPtr) == -1)
            return allPackets;
        
        return prepareSendPayload(sensorPtr);
    }

    inline void checkIfSensorWantSendPacket()
    {
        for (int i = 0; i < knownSensors.size(); i++)
        {
            Sensor* ptr = nullptr;
            knownSensors.valueAt(i, ptr);
            if(ptr == nullptr)
                continue;
            if(!ptr->status.statusReg.isSensorInfoKnown && !ptr->status.statusReg.isSensorInfoKnownSend)
            {
                uint32_t tmp = ptr->info.identifier;
                log("Sensor", LogLevel::Debug, "Sensor want send info request id: 0x%lx", tmp);
                sensorsWantToSendData.push(ptr);
                ptr->status.statusReg.isSensorInfoKnownSend = 1;
                continue;
            }
            
            if(!ptr->status.statusReg.isSensorInfoKnown && ptr->status.statusReg.isSensorInfoKnownSend)
                continue;
            // if(!ptr->status.statusReg.isDone)
            // {
            //     log("Sensor", LogLevel::Debug, "Sensor want send data id: 0x%x", ptr->info.identifier);
            //     sensorsWantToSendData.push(ptr);
            //     continue;
            // }

            // if(ptr->status.sendDataPackets != ptr->status.recvDataPackets)
            if(ptr->status.getCountOfSendDataPackets() < ptr->info.dataPacketsMaxCount && 
               ptr->status.getCountOfRecvDataPackets() < ptr->info.dataPacketsMaxCount)
            {
                log("Sensor", LogLevel::Debug, "tmp %lx %lx", ptr->status.sendDataPackets, ptr->status.recvDataPackets);
                log("Sensor", LogLevel::Debug, "Sensor want send data request id: 0x%lx", ptr->info.identifier);
                sensorsWantToSendData.push(ptr);
                continue;
            }

            // if(ptr->status.sendCalibPackets != ptr->status.recvCalibPackets)
            if(ptr->status.getCountOfSendCalibPackets() < ptr->info.dataPacketsMaxCount &&
               ptr->status.getCountOfRecvCalibPackets() < ptr->info.calibrationPacketsMaxCount)
            {
                log("Sensor", LogLevel::Debug, "Sensor want send calib request id: 0x%lx", ptr->info.identifier);
                sensorsWantToSendData.push(ptr);
                continue; 
            }
        }
    }

    inline void processSensorsRawData()
    {
        for (int i = 0; i < knownSensors.size(); i++)
        {
            Sensor *sensor = nullptr;
            knownSensors.valueAt(i, sensor);
            if(sensor == nullptr)
                return;
            // ESP_LOGI("SENSOR", "0");
            
            if(!sensor->status.statusReg.isSensorInfoKnown)
                continue;
            if(!sensor->status.statusReg.isSensorCalibReceived)
                continue;
            if(!sensor->status.statusReg.isSensorDataReceived)
                continue;
            if(sensor->status.statusReg.isDataCalculated)
                continue;
            
            ESP_LOGI("SENSOR", "TTTT");
            
            int result = sensor->calculateData();
            if(result == 0)
            {
                log("Sensor", LogLevel::Info, "%s: Calculating done", getSensorTypeName(sensor->getSensorType()));
            }
            else
            {
                log("Sensor", LogLevel::Info, "%s: Calculating error", getSensorTypeName(sensor->getSensorType()));
            }

            // // TODO do proper calculation for proper sensor
            // switch (sensor->info.sensorType)
            // {
            // case SensorType::SimpleWeatherSensorBMP280:
            //     log("Sensor", LogLevel::Info, "Calculating SimpleWeatherSensorBMP280");
            //     calculateBMP280(sensor->calibrationData, sensor->lastSensorPackets, sensor->calculatedData);
            //     break;
            // case SensorType::SimpleWeatherSensorBME280:
            //     log("Sensor", LogLevel::Info, "Calculating SimpleWeatherSensorBME280");
            //     calculateBME280(sensor->calibrationData, sensor->lastSensorPackets, sensor->calculatedData);
            //     break;
            // default:
            //     log("Sensor", LogLevel::Error, "Error unknow sensor type");
            //     break;
            // }

            sensor->status.statusReg.isDataCalculated = 1;
        }
    }
}
