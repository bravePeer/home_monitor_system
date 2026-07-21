#pragma once
#include "general/platform_error.hpp"
#include "sensor/sensor/sensor_common.hpp"
#include "packet.hpp"

#include "esp_log.h"

namespace sensor
{
    // inline ErrorCode calculateBMP280Appendix(
    //     SensorCalibrationData& calib, 
    //     RingBuffer<sensorPacket::SensorPacketWithLen, SensorRawDataCount>& rawPackets, 
    //     RingBuffer<SensorSingleData, SensorDataCount>& calculatedData)
    // {
    //     uint64_t timestamp = getTime();
    //
    //     sensorPacket::SensorPacketWithLen packet;
    //     rawPackets.pop(packet);
    //     if(packet.packet.Data.header.type == sensorPacket::PacketType::SensorData0)
    //     {
    //         /// @todo add some checking
    //     }
// 
    //     int32_t adcT = bytesToInt32(packet.packet.Data.temperatureRaw) >> 4;
// 
    //     int32_t temperatureFine = 0;
// 
    //     // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123”equals 51.23 DegC. 
    //     // temperatureFine carries fine temperature as global value
    //     int32_t var1T  = ((((adcT >> 3) - ((int32_t)calib.bme280.digT1 << 1))) * ((int32_t)calib.bme280.digT2)) >> 11;
    //     int32_t var2T  = (((((adcT >> 4) - ((int32_t)calib.bme280.digT1)) * ((adcT >> 4) - ((int32_t)calib.bme280.digT1))) >> 12) * ((int32_t)calib.bme280.digT3)) >> 14;
    //     temperatureFine = var1T + var2T;
    //     int32_t temperature  = (temperatureFine * 5 + 128) >> 8; // in 0.01 degC
// 
    //     SensorSingleData temperatureData = {
    //         .dataType = SensorSingleDataType::Temperature,
    //         .timestamp = timestamp,
    //         .temperature = temperature
    //     };
    //     calculatedData.push(temperatureData);
//
    //     int32_t adcP = bytesToInt32(packet.packet.Data.pressureRaw) >> 4;
//
    //     // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    //     // Output value of “24674867”represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    //     int64_t var1P = ((int64_t)temperatureFine) - 128000;
    //     int64_t var2P = var1P * var1P * (int64_t)calib.bme280.digP6;
    //     var2P = var2P + ((var1P*(int64_t)calib.bme280.digP5)<<17);
    //     var2P = var2P + (((int64_t)calib.bme280.digP4)<<35);
    //     var1P = ((var1P * var1P * (int64_t)calib.bme280.digP3)>>8) + ((var1P * (int64_t)calib.bme280.digP2)<<12);
    //     var1P = (((((int64_t)1)<<47)+var1P))*((int64_t)calib.bme280.digP1)>>33;
    //     if(var1P == 0)
    //     {
    //         return ErrorCode::Fail; // avoid exception caused by division by zero
    //     }
    //     int64_t pressure = 1048576 - adcP;
    //     pressure = (((pressure << 31) - var2P) * 3125) / var1P;
    //     var1P = (((int64_t)calib.bme280.digP9) * (pressure>>13) * (pressure>>13)) >> 25;
    //     var2P = (((int64_t)calib.bme280.digP8) * pressure) >> 19;
    //     pressure = ((pressure + var1P + var2P) >> 8) + (((int64_t)calib.bme280.digP7)<<4);
    //     pressure = (pressure * 10) / 256; // result is 12312 = 1231.2 Pa
// 
    //     SensorSingleData pressureData = {
    //         .dataType = SensorSingleDataType::Pressure,
    //         .timestamp = timestamp,
    //         .pressure = static_cast<uint32_t>(pressure)
    //     };
    //     calculatedData.push(pressureData);
    //     ESP_LOGI("CALC", "Temperature: %d, Pressure: %d, TemperatureFine: %d", (int)temperature, (int)pressure, (int)temperatureFine);
// 
    //     return ErrorCode::Ok;
    // }

    inline ErrorCode calculateBMP280(
        SensorCalibrationData& calib, 
        RingBuffer<sensorPacket::SensorPacketWithLen, SensorRawDataCount>& rawPackets, 
        RingBuffer<SensorSingleData, SensorDataCount>& calculatedData)
    {
        uint64_t timestamp = getTime();

        sensorPacket::SensorPacketWithLen packet;
        rawPackets.pop(packet);
        if(packet.packet.Data.header.type == sensorPacket::PacketType::SensorData0)
        {
            /// @todo add some checking
        }

        int32_t adcT = bytesToInt32(packet.packet.Data.temperatureRaw) >> 4;

        int32_t temperatureFine = 0;

        // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123”equals 51.23 DegC. 
        // temperatureFine carries fine temperature as global value
        int32_t var1T  = ((((adcT>>3) - ((int32_t)calib.bme280.digT1<<1))) * ((int32_t)calib.bme280.digT2)) >> 11;
        int32_t var2T  = (((((adcT>>4) - ((int32_t)calib.bme280.digT1)) * ((adcT>>4) - ((int32_t)calib.bme280.digT1))) >> 12) * ((int32_t)calib.bme280.digT3)) >> 14;
        temperatureFine = var1T + var2T;
        int32_t temperature  = (temperatureFine * 5 + 128) >> 8; // in 0.01 degC

        SensorSingleData temperatureData = {
            .dataType = SensorSingleDataType::Temperature,
            .timestamp = timestamp,
            .temperature = temperature
        };
        calculatedData.push(temperatureData);

        int32_t adcP = bytesToInt32(packet.packet.Data.pressureRaw) >> 4;

        // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
        // Output value of “24674867”represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        int64_t var1P = ((int64_t)temperatureFine) - 128000;
        int64_t var2P = var1P * var1P * (int64_t)calib.bme280.digP6;
        var2P = var2P + ((var1P*(int64_t)calib.bme280.digP5)<<17);
        var2P = var2P + (((int64_t)calib.bme280.digP4)<<35);
        var1P = ((var1P * var1P * (int64_t)calib.bme280.digP3)>>8) + ((var1P * (int64_t)calib.bme280.digP2)<<12);
        var1P = (((((int64_t)1)<<47)+var1P))*((int64_t)calib.bme280.digP1)>>33;
        if(var1P == 0)
        {
            return ErrorCode::Fail; // avoid exception caused by division by zero
        }
        int64_t pressure = 1048576 - adcP;
        pressure = (((pressure << 31) - var2P) * 3125) / var1P;
        var1P = (((int64_t)calib.bme280.digP9) * (pressure>>13) * (pressure>>13)) >> 25;
        var2P = (((int64_t)calib.bme280.digP8) * pressure) >> 19;
        pressure = ((pressure + var1P + var2P) >> 8) + (((int64_t)calib.bme280.digP7)<<4);
        pressure = (pressure * 10) / 256; // result is 12312 = 1231.2 Pa

        SensorSingleData pressureData = {
            .dataType = SensorSingleDataType::Pressure,
            .timestamp = timestamp,
            .pressure = static_cast<uint32_t>(pressure)
        };
        calculatedData.push(pressureData);
        ESP_LOGI("CALC", "Temperature: %d, Pressure: %d, TemperatureFine: %d", (int)temperature, (int)pressure, (int)temperatureFine);

        return ErrorCode::Ok;
    }

    inline ErrorCode calculateBME280(
        SensorCalibrationData& calib, 
        RingBuffer<sensorPacket::SensorPacketWithLen, SensorRawDataCount>& rawPackets, 
        RingBuffer<SensorSingleData, SensorDataCount>& calculatedData)
    {
        sensorPacket::SensorPacketWithLen packet;
        rawPackets.pop(packet);
        if(packet.packet.Data.header.type == sensorPacket::PacketType::SensorData0)
        {
            /// @todo
        }

        uint64_t timestamp = getTime();

        /// Temperature calculation
        int32_t adcT = bytesToInt32(packet.packet.Data.temperatureRaw) >> 4;
        // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
        // temperatureFine carries fine temperature as global value 
        int32_t temperatureFine = 0;
        int32_t var1T  = ((((adcT>>3) - ((int32_t)calib.bme280.digT1<<1))) * ((int32_t)calib.bme280.digT2)) >> 11;
        int32_t var2T  = (((((adcT>>4) - ((int32_t)calib.bme280.digT1)) * ((adcT>>4) - ((int32_t)calib.bme280.digT1))) >> 12) *  ((int32_t)calib.bme280.digT3)) >> 14;    
        temperatureFine = var1T + var2T;    
        int32_t temperature  = (temperatureFine * 5 + 128) >> 8;    

        SensorSingleData temperatureData = {
            .dataType = SensorSingleDataType::Temperature,
            .timestamp = timestamp,
            .temperature = temperature
        };
        calculatedData.push(temperatureData);

        /// Pressure calculation
        int32_t adcP = bytesToInt32(packet.packet.Data.pressureRaw) >> 4;
        // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
        // Output value of “24674867”represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        int64_t var1P = ((int64_t)temperatureFine) - 128000;
        int64_t var2P = var1P * var1P * (int64_t)calib.bme280.digP6;
        var2P = var2P + ((var1P*(int64_t)calib.bme280.digP5)<<17);
        var2P = var2P + (((int64_t)calib.bme280.digP4)<<35);
        var1P = ((var1P * var1P * (int64_t)calib.bme280.digP3)>>8) + ((var1P * (int64_t)calib.bme280.digP2)<<12);
        var1P = ((((int64_t)1)<<47)+var1P)*((int64_t)calib.bme280.digP1)>>33;
        if(var1P == 0)
        {
            return ErrorCode::Fail; // avoid exception caused by division by zero
        }
        int64_t pressure = 1048576 - adcP;
        pressure = (((pressure << 31) - var2P) * 3125) / var1P;
        var1P = (((int64_t)calib.bme280.digP9) * (pressure>>13) * (pressure>>13)) >> 25;
        var2P = (((int64_t)calib.bme280.digP8) * pressure) >> 19;
        pressure = ((pressure + var1P + var2P) >> 8) + (((int64_t)calib.bme280.digP7)<<4);
        pressure = (pressure * 10) / 256; // result is 12312 = 1231.2 Pa

        SensorSingleData pressureData = {
            .dataType = SensorSingleDataType::Pressure,
            .timestamp = timestamp,
            .pressure = static_cast<uint32_t>(pressure)
        };
        calculatedData.push(pressureData);

        /// Humidity calculation
        int32_t adcH = bytesToInt32(packet.packet.Data.humidityRaw);
        
        int32_t varH = (temperatureFine - ((int32_t)76800));
        varH = (((((adcH << 14) - (((int32_t)calib.bme280.digH4) << 20) - (((int32_t)calib.bme280.digH5) * varH)) + ((int32_t)16384)) >> 15) 
                * (((((((varH * ((int32_t)calib.bme280.digH6)) >> 10) * (((varH * ((int32_t)calib.bme280.digH3)) >> 11) + ((int32_t)32768))) >> 10) 
                + ((int32_t)2097152)) * ((int32_t)calib.bme280.digH2) + 8192) >> 14));
        
        varH = (varH - (((((varH >> 15) * (varH >> 15)) >> 7) * ((int32_t)calib.bme280.digH1)) >> 4));    
        varH = (varH < 0 ? 0 : varH);     
        varH = (varH > 419430400 ? 419430400 : varH);        
        uint32_t humidity = (uint32_t)(varH>>12); // Q22.10 format (22 integer and 10 fractional bits).  Humidity in %RH = humidity / 1024.0f
        humidity = (humidity * 1000) / 1024; // in 0.001 %RH

        SensorSingleData humidityData = {
            .dataType = SensorSingleDataType::Humidity,
            .timestamp = timestamp,
            .pressure = static_cast<uint16_t>(humidity)
        };
        calculatedData.push(humidityData);

        /// Battery voltage calculation
        uint32_t adcV = bytesToInt32(packet.packet.Data.batteryVoltageRaw) >> 2;
        uint32_t voltage = ((static_cast<uint32_t>(calib.resistorR1Value) + static_cast<uint32_t>(calib.resistorR2Value)) * adcV * 1000) 
                            / static_cast<uint32_t>(calib.resistorR2Value); // mV

        SensorSingleData voltageData = {
            .dataType = SensorSingleDataType::Voltage,
            .timestamp = timestamp,
            .voltage = static_cast<uint16_t>(voltage)
        };
        calculatedData.push(voltageData);

        ESP_LOGI("CALC", "Calib H: %d, %d, %d, %d, %d, %d", (int)calib.bme280.digH1, (int)calib.bme280.digH2, (int)calib.bme280.digH3, (int)calib.bme280.digH4, (int)calib.bme280.digH5, (int)calib.bme280.digH6);

        ESP_LOGI("CALC", "adcT: %d, adcP: %d, adcH: %d, adcV: %d", (int)adcT, (int)adcP, (int)adcH, (int)adcV);
        ESP_LOGI("CALC", "Temperature: %d, Pressure: %d, Humidity: %d TemperatureFine: %d Voltage %d", (int)temperature, (int)pressure, (int)humidity, (int)temperatureFine, (int)voltage);

        return ErrorCode::Ok;
    }

}