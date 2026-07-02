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

namespace sensor
{
    inline const char* getSensorTypeName(SensorType type)
    {
        switch (type)
        {
        case SensorType::Test:
            return "Test";
        case SensorType::SimpleWeatherSensorBMP280:
            return "SimpleWeatherSensorBMP280";
        case SensorType::SimpleWeatherSensorBME280:
            return "SimpleWeatherSensorBME280";
        case SensorType::SimpleWeatherSensorBME680:
            return "SimpleWeatherSensorBME680";
        default:
            return "Unknown";
        }
    }

    inline void dumpSensorInfo(const Sensor* sensor)
    {
        printf("------------ Sensor info dump -----------\n");
        printf("Identifier: 0x%04lx\n", sensor->info.identifier);
        printf("Address: %02x:%02x:%02x:%02x:%02x\n", sensor->info.address[0], sensor->info.address[1], sensor->info.address[2], sensor->info.address[3], sensor->info.address[4]);
        printf("Initialization time: %llu\n", sensor->info.initializationTime);
        printf("Software version: 0x%04lx\n", sensor->info.softwareVersion);
        printf("Hardware version: 0x%04lx\n", sensor->info.hardwareVersion);
        printf("Sensor type: %s (0x%04lx)\n", getSensorTypeName(sensor->info.sensorType), static_cast<uint32_t>(sensor->info.sensorType));
        printf("Data packets max count: %d\n", sensor->info.dataPacketsMaxCount);
        printf("Calibration packets max count: %d\n", sensor->info.calibrationPacketsMaxCount);
        printf("-----------------------------------------\n");
    }

    inline void dumpSensorCalibrationData(const Sensor* sensor)
    {
        printf("----- Sensor calibration data dump ------\n");
        printf("Identifier: 0x%04lx\n", sensor->info.identifier);
        printf("Calibration data raw: ");
        for(int i = 0; i < 128; i++)
        {
            printf("%02x ", sensor->calibrationData.raw[i]);
        }
        printf("\n");
        printf("-----------------------------------------\n");
    }
}