#include <cstring>
#include <gtest/gtest.h>
#include "sensor/sensor/sensor.hpp"

class SensorTest : public testing::Test
{
public:
    SensorTest()
    {
        resetGlobals();

        packetInfoSensorId0.packet.Info.header.direction = sensorPacket::PacketDirection::Report;
        packetInfoSensorId0.packet.Info.header.errorFlag = sensorPacket::PacketError::NoError;
        packetInfoSensorId0.packet.Info.header.type = sensorPacket::PacketType::SensorInfo;
        packetInfoSensorId0.packet.Info.hardwareVersionValue = 1;
        packetInfoSensorId0.packet.Info.softwareVersionValue = 2;
        packetInfoSensorId0.packet.Info.crc = 0;
        packetInfoSensorId0.packet.Info.identifierValue = 0;
        packetInfoSensorId0.packet.Info.sensorType = 0;
        packetInfoSensorId0.packet.Info.calibrationPacketsMaxCount = 0;
        packetInfoSensorId0.packet.Info.dataPacketsMaxCount = 0;
        packetInfoSensorId0.size = 17;

        packetInfoSensorId1.packet.Info.header.direction = sensorPacket::PacketDirection::Report;
        packetInfoSensorId1.packet.Info.header.errorFlag = sensorPacket::PacketError::NoError;
        packetInfoSensorId1.packet.Info.header.type = sensorPacket::PacketType::SensorInfo;
        packetInfoSensorId1.packet.Info.hardwareVersionValue = 1;
        packetInfoSensorId1.packet.Info.softwareVersionValue = 1;
        packetInfoSensorId1.packet.Info.crc = 0;
        packetInfoSensorId1.packet.Info.identifierValue = 1;
        packetInfoSensorId1.packet.Info.sensorType = 1;
        packetInfoSensorId1.packet.Info.calibrationPacketsMaxCount = 1;
        packetInfoSensorId1.packet.Info.dataPacketsMaxCount = 2;
        packetInfoSensorId1.size = 17;

        packetInfoSensorId2.packet.Info.header.direction = sensorPacket::PacketDirection::Report;
        packetInfoSensorId2.packet.Info.header.errorFlag = sensorPacket::PacketError::NoError;
        packetInfoSensorId2.packet.Info.header.type = sensorPacket::PacketType::SensorInfo;
        packetInfoSensorId2.packet.Info.hardwareVersionValue = 2;
        packetInfoSensorId2.packet.Info.softwareVersionValue = 2;
        packetInfoSensorId2.packet.Info.crc = 0;
        packetInfoSensorId2.packet.Info.identifierValue = 2;
        packetInfoSensorId2.packet.Info.sensorType = 2;
        packetInfoSensorId2.packet.Info.calibrationPacketsMaxCount = 3;
        packetInfoSensorId2.packet.Info.dataPacketsMaxCount = 3;
        packetInfoSensorId2.size = 17;
        sensorPacket::generateCrc(packetInfoSensorId0);
    }

    void resetGlobals()
    {
        sensor::knownSensors.clear();
        sensor::sensorsWantToSendData.clear();
    }

    /// @brief Adds fake sensors
    void addSensors(const std::vector<sensor::Sensor>& sensors)
    {
        for (auto sensor: sensors)
        {
            sensor::knownSensors.add(sensor);
        }
    }

    sensorPacket::SensorPacketWithLen packetInfoSensorId0;
    sensorPacket::SensorPacketWithLen packetInfoSensorId1;
    sensorPacket::SensorPacketWithLen packetInfoSensorId2;
};

TEST_F(SensorTest, RecvInfoPacket)
{
    ASSERT_EQ(sensor::knownSensors.size(), 0); // Not known any sensor
    ASSERT_EQ(sensor::processRecvPayload(packetInfoSensorId0), 0);

    ASSERT_EQ(sensor::knownSensors.size(), 1); // Known 1 sensor
    ASSERT_EQ(sensor::processRecvPayload(packetInfoSensorId0), 0);
    ASSERT_EQ(sensor::knownSensors.size(), 1); // Still Known 1 sensor

    ASSERT_EQ(sensor::processRecvPayload(packetInfoSensorId1), 0);
    ASSERT_EQ(sensor::knownSensors.size(), 2); // Known 2 sensor
    ASSERT_EQ(sensor::processRecvPayload(packetInfoSensorId2), 0);
    ASSERT_EQ(sensor::knownSensors.size(), 3); // Known 3 sensor
    ASSERT_EQ(sensor::processRecvPayload(packetInfoSensorId1), 0);
    ASSERT_EQ(sensor::knownSensors.size(), 3); // Still Known 3 
    
    std::array<sensorPacket::SensorPacketWithLen*, 3> packets = { 
        &packetInfoSensorId0, &packetInfoSensorId1, &packetInfoSensorId2
    };

    for(int i = 0; i < packets.size(); i++)
    {

        sensor::Sensor* sensorPtr = nullptr;
        int ret = sensor::knownSensors.valueByExpression([](sensor::Sensor& refSensor, void* args)->int {
            uint32_t* id = reinterpret_cast<uint32_t*>(args);
            if(refSensor.info.identifier == *id)
                return 0;
            return -1;
        }, sensorPtr, &packets[i]->packet.Info.identifierValue);

        ASSERT_EQ(ret, 0);

        ASSERT_NE(sensorPtr, nullptr) << "Pointer points to nullptr instead of sensor";
        ASSERT_EQ(sensorPtr->info.identifier, packets[i]->packet.Info.identifierValue) 
            << "Sensor " << i << ": wrong id";
        ASSERT_EQ(sensorPtr->info.hardwareVersion, packets[i]->packet.Info.hardwareVersionValue) 
            << "Sensor " << i << ": wrong hardware version";
        ASSERT_EQ(sensorPtr->info.softwareVersion, packets[i]->packet.Info.softwareVersionValue)
            << "Sensor " << i << ": wrong software version";
        ASSERT_EQ(static_cast<uint32_t>(sensorPtr->info.sensorType), packets[i]->packet.Info.sensorType)
            << "Sensor " << i << ": wrong sensor type";
        ASSERT_EQ(sensorPtr->info.dataPacketsMaxCount, packets[i]->packet.Info.dataPacketsMaxCount)
            << "Sensor " << i << ": wrong data packets max count";
        ASSERT_EQ(sensorPtr->info.calibrationPacketsMaxCount, packets[i]->packet.Info.calibrationPacketsMaxCount)
            << "Sensor " << i << ": wrong calibration packets max count";
        ASSERT_TRUE(sensorPtr->status.statusReg.isSensorInfoKnown)
            << "Sensor " << i << ": has not set flag isSensorInfoKnown";
    }

    // TODO Add when knownSensors is full
    
}

TEST_F(SensorTest, RecvCalibPacket)
{
    {
        sensor::Sensor sensor0;
        sensor0.info.calibrationPacketsMaxCount = 0;
        sensor0.info.sensorType = sensor::SensorType::SimpleWeatherStationBME280;
        sensor0.info.identifier = 0;
        sensor::Sensor sensor1;
        sensor1.info.calibrationPacketsMaxCount = 1;
        sensor1.info.identifier = 1;
        sensor::Sensor sensor2;
        sensor2.info.calibrationPacketsMaxCount = 3;
        sensor2.info.identifier = 2;

        std::vector<sensor::Sensor> sensors = {sensor0};
        addSensors(sensors);
    }
    
    {
        sensorPacket::SensorPacketWithLen packet0;              
        packet0.packet.CalibData.header.direction = sensorPacket::PacketDirection::Report;
        packet0.packet.CalibData.header.errorFlag = sensorPacket::PacketError::NoError;
        packet0.packet.CalibData.header.type = sensorPacket::PacketType::SensorCalibData0;
        packet0.packet.CalibData.crc = 0;
        packet0.packet.CalibData.identifierValue = 0;
        packet0.packet.CalibData.calibData[0] = {13};
        packet0.size = 32;

        ASSERT_EQ(sensor::processRecvPayload(packet0), 2) << "Test if sensor not contain calibration data";

        sensor::Sensor tsensor;
        tsensor.info.sensorType = sensor::SensorType::Test;
        tsensor.info.calibrationPacketsMaxCount = 0;
        tsensor.info.identifier = 0;
        ASSERT_EQ(sensor::processRecvCalib(packet0, &tsensor), 2) << "Sensor not contain calibration data";

        tsensor.info.calibrationPacketsMaxCount = 1;
        ASSERT_EQ(sensor::processRecvCalib(packet0, &tsensor), 0) << "Sensor not process calibration data";
        ASSERT_EQ(tsensor.calibrationData.raw[0], packet0.packet.CalibData.calibData[0]) << "Wrong calibration data";
    }
}

TEST_F(SensorTest, TestSimpleWeatherStationBMP280)
{
    // Init packet
    
}
