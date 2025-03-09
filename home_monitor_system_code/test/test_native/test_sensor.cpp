#include <gtest/gtest.h>
#include <rp2040/wireless_communicator/wireless_communicator.hpp>

class ProcessSensorPayloadTest : public testing::Test
{
    protected:
    ProcessSensorPayloadTest()
    {
        packetInfoId0.Info.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorInfo
        };
        packetInfoId0.Info.hardwareVersionValue = 1;
        packetInfoId0.Info.softwareVersionValue = 2;
        packetInfoId0.Info.crc = 0;
        packetInfoId0.Info.identifierValue = 0;
        packetInfoId0.Info.sensorType = 0;

        packetInfoId1.Info.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorInfo
        };
        packetInfoId1.Info.hardwareVersionValue = 0;
        packetInfoId1.Info.softwareVersionValue = 0;
        packetInfoId1.Info.crc = 0;
        packetInfoId1.Info.identifierValue = 1;
        packetInfoId1.Info.sensorType = 0;

        packetInfoId2.Info.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorInfo
        };
        packetInfoId2.Info.hardwareVersionValue = 0;
        packetInfoId2.Info.softwareVersionValue = 0;
        packetInfoId2.Info.crc = 0;
        packetInfoId2.Info.identifierValue = 2;
        packetInfoId2.Info.sensorType = 0;

        packetInfoId3.Info.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorInfo
        };
        packetInfoId3.Info.hardwareVersionValue = 0;
        packetInfoId3.Info.softwareVersionValue = 0;
        packetInfoId3.Info.crc = 0;
        packetInfoId3.Info.identifierValue = 3;
        packetInfoId3.Info.sensorType = 0;
    
        packetData0Id1.Data.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorData
        };
        packetData0Id1.Data.identifierValue = 1;
        packetData0Id1.Data.crc = 0;
        packetData0Id1.Data.humidityValue = 1;
        packetData0Id1.Data.pressureValue = 2;
        packetData0Id1.Data.temperatureValue = 3;
        packetData0Id1.Data.batteryVoltageValue = 10;

        packetData1Id1.Data.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorData
        };
        packetData1Id1.Data.identifierValue = 1;
        packetData1Id1.Data.crc = 0;
        packetData1Id1.Data.humidityValue = 2;
        packetData1Id1.Data.pressureValue = 2;
        packetData1Id1.Data.temperatureValue = 2;
        packetData1Id1.Data.batteryVoltageValue = 20;

        packetData2Id1.Data.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorData
        };
        packetData2Id1.Data.identifierValue = 1;
        packetData2Id1.Data.crc = 0;
        packetData2Id1.Data.humidityValue = 3;
        packetData2Id1.Data.pressureValue = 3;
        packetData2Id1.Data.temperatureValue = 3;
        packetData2Id1.Data.batteryVoltageValue = 30;


        packetBadCrc.Data.header = {
            .direction = sensorPacket::PacketDirection::Response,
            .errorFlag = 0,
            .type = sensorPacket::PacketType::SensorData
        };
        packetBadCrc.Data.identifierValue = 5;
        packetBadCrc.Data.crc = 0;
        packetBadCrc.Data.humidityValue = 5;
        packetBadCrc.Data.pressureValue = 5;
        packetBadCrc.Data.temperatureValue = 5;
        packetBadCrc.Data.batteryVoltageValue = 30;

        sensorPacket::generateCrc(packetInfoId0.raw, 32);
        sensorPacket::generateCrc(packetInfoId1.raw, 32);
        sensorPacket::generateCrc(packetInfoId2.raw, 32);
        sensorPacket::generateCrc(packetInfoId3.raw, 32);

        sensorPacket::generateCrc(packetData0Id1.raw, 32);
        sensorPacket::generateCrc(packetData1Id1.raw, 32);
        sensorPacket::generateCrc(packetData2Id1.raw, 32);
    }

    sensorPacket::SensorPacket packetInfoId0;
    sensorPacket::SensorPacket packetInfoId1;
    sensorPacket::SensorPacket packetInfoId2;
    sensorPacket::SensorPacket packetInfoId3;

    sensorPacket::SensorPacket packetData0Id1;
    sensorPacket::SensorPacket packetData1Id1;
    sensorPacket::SensorPacket packetData2Id1;

    sensorPacket::SensorPacket packetBadCrc;
};

TEST_F(ProcessSensorPayloadTest, ReceivedDiffrentIdentifiers)
{
    auto checker = [](sensor::Sensor& sensor, sensorPacket::SensorPacket& packetToCheck){
        ASSERT_EQ(sensor.sensorInfo.identifier, packetToCheck.Info.identifierValue);
        ASSERT_EQ(sensor.sensorInfo.softwareVersion, packetToCheck.Info.softwareVersionValue);
        ASSERT_EQ(sensor.sensorInfo.hardwareVersion, packetToCheck.Info.hardwareVersionValue);
        ASSERT_EQ(sensor.sensorInfo.sensorType, static_cast<sensor::SensorType>(packetToCheck.Info.sensorType));
    };

    sensor::Sensor* tmpSensor = nullptr;

    // No known sensors
    ASSERT_EQ(sensor::knownSensors.valueAt(0, tmpSensor), -1);

    // First Info
    sensor::processSensorPayload(packetInfoId0.raw, 32);
    ASSERT_EQ(sensor::knownSensors.valueAt(0, tmpSensor), 0);
    checker(*tmpSensor, packetInfoId0);
    ASSERT_EQ(sensor::knownSensors.valueAt(1, tmpSensor), -1);

    sensor::processSensorPayload(packetInfoId1.raw, 32);
    ASSERT_EQ(sensor::knownSensors.valueAt(0, tmpSensor), 0);
    checker(*tmpSensor, packetInfoId1);
    ASSERT_EQ(sensor::knownSensors.valueAt(2, tmpSensor), -1);

    sensor::processSensorPayload(packetInfoId2.raw, 32);
    ASSERT_EQ(sensor::knownSensors.valueAt(0, tmpSensor), 0);
    checker(*tmpSensor, packetInfoId2);
    ASSERT_EQ(sensor::knownSensors.valueAt(3, tmpSensor), -1);

    sensor::processSensorPayload(packetInfoId3.raw, 32);
    ASSERT_EQ(sensor::knownSensors.valueAt(0, tmpSensor), 0);
    checker(*tmpSensor, packetInfoId3);
    ASSERT_EQ(sensor::knownSensors.valueAt(4, tmpSensor), -1);
}

TEST_F(ProcessSensorPayloadTest, ReceivedInfoAndData)
{
    sensor::processSensorPayload(packetInfoId0.raw, 32);
    sensor::processSensorPayload(packetInfoId1.raw, 32);
    sensor::processSensorPayload(packetInfoId2.raw, 32);
    sensor::processSensorPayload(packetInfoId3.raw, 32);

    sensor::Sensor* ptrSensor = nullptr;
    // Send data to sensor id = 1
    sensor::knownSensors.valueByExpression([](sensor::Sensor& sensor, void* args)->int {
        if(sensor.sensorInfo.identifier == *reinterpret_cast<uint32_t*>(args))
            return 0;
        return -1;
    }, 
    ptrSensor, &packetInfoId1.General.identifierValue);

    ASSERT_EQ(ptrSensor->sensorInfo.identifier, packetData0Id1.General.identifierValue) << "Sensor id is wrong";

    ASSERT_EQ(ptrSensor->sensorData.getDataCount(), 0);
    sensor::processSensorPayload(packetData0Id1.raw, 32);
    ASSERT_EQ(ptrSensor->sensorData.getDataCount(), 1);

    sensor::processSensorPayload(packetData1Id1.raw, 32);
    sensor::processSensorPayload(packetData2Id1.raw, 32);
    ASSERT_EQ(ptrSensor->sensorData.getDataCount(), 3) << "Size of ring buffer is wrong";

    auto checker = [](sensor::SensorData& sensorData, sensorPacket::SensorPacket& packet) {
        ASSERT_EQ(sensorData.packet.Data.temperatureValue, packet.Data.temperatureValue);
        ASSERT_EQ(sensorData.packet.Data.pressureValue, packet.Data.pressureValue);
        ASSERT_EQ(sensorData.packet.Data.humidityValue, packet.Data.humidityValue);
        ASSERT_EQ(sensorData.packet.Data.batteryVoltageValue, packet.Data.batteryVoltageValue);
    };

    sensor::SensorData sensorData;
    ASSERT_EQ(ptrSensor->sensorData.popData(sensorData), 0);
    checker(sensorData, packetData0Id1);
    ASSERT_EQ(ptrSensor->sensorData.getDataCount(), 2);
    
    ASSERT_EQ(ptrSensor->sensorData.popData(sensorData), 0);
    checker(sensorData, packetData1Id1);
    
    ASSERT_EQ(ptrSensor->sensorData.popData(sensorData), 0);
    checker(sensorData, packetData2Id1);

    ASSERT_EQ(ptrSensor->sensorData.getDataCount(), 0);
}

TEST_F(ProcessSensorPayloadTest, CrcGeneration)
{
    uint8_t crcSum = packetBadCrc.raw[0];
    for (uint8_t i = 2; i < 32; i++)
        crcSum += packetBadCrc.raw[i];
    
    ASSERT_EQ(sensorPacket::checkCrc(packetBadCrc.raw, 32), -1);

    ASSERT_EQ(sensorPacket::generateCrc(packetBadCrc.raw, 32), 0);
    ASSERT_EQ(crcSum, packetBadCrc.raw[1]) << "Generated crc is wrong!";
    ASSERT_EQ(sensorPacket::checkCrc(packetBadCrc.raw, 32), 0) << "Checking crc is wrong!";
}