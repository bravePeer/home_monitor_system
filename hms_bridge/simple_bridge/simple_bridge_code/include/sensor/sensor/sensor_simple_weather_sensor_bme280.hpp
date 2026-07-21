#pragma once

#include "sensor/sensor/sensor.hpp"
#include <cinttypes>

namespace sensor
{
    
    inline ErrorCode calculateBME280(
        SensorCalibrationData& calib, 
        RingBuffer<sensorPacket::SensorPacketWithLen, SensorRawDataCount>& rawPackets, 
        RingBuffer<SensorSingleData, SensorDataCount>& calculatedData)
    {
        
    }

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
            // this->lastSensorPackets

            const CalibrationData2& calib = this->calibrationData;
            RingBuffer<sensorPacket::SensorPacketWithLen, SensorRawDataCount>& rawPackets = this->lastSensorPackets;
            // const RingBuffer<SensorSingleData, SensorDataCount>& calculatedData = this.

            /// @todo check if calibration data, sensor info and raw data is known

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
            actualCalculatedData.temperature = static_cast<int32_t>(temperature);

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
                return -1; // avoid exception caused by division by zero
            }
            int64_t pressure = 1048576 - adcP;
            pressure = (((pressure << 31) - var2P) * 3125) / var1P;
            var1P = (((int64_t)calib.bme280.digP9) * (pressure>>13) * (pressure>>13)) >> 25;
            var2P = (((int64_t)calib.bme280.digP8) * pressure) >> 19;
            pressure = ((pressure + var1P + var2P) >> 8) + (((int64_t)calib.bme280.digP7)<<4);
            pressure = (pressure * 10) / 256; // result is 12312 = 1231.2 Pa
            actualCalculatedData.pressure = static_cast<uint32_t>(pressure);
            
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
            actualCalculatedData.humidity =static_cast<uint16_t>(humidity);

            /// Battery voltage calculation
            uint32_t adcV = bytesToInt32(packet.packet.Data.batteryVoltageRaw) >> 2;
            uint32_t voltage = ((static_cast<uint32_t>(calib.resistorR1Value) + static_cast<uint32_t>(calib.resistorR2Value)) * adcV * 1000) 
                                / static_cast<uint32_t>(calib.resistorR2Value); // mV
            actualCalculatedData.voltage = static_cast<uint16_t>(voltage);

            ESP_LOGI("CALC", "Calib H: %d, %d, %d, %d, %d, %d", (int)calib.bme280.digH1, (int)calib.bme280.digH2, (int)calib.bme280.digH3, (int)calib.bme280.digH4, (int)calib.bme280.digH5, (int)calib.bme280.digH6);

            ESP_LOGI("CALC", "adcT: %d, adcP: %d, adcH: %d, adcV: %d", (int)adcT, (int)adcP, (int)adcH, (int)adcV);
            ESP_LOGI("CALC", "Temperature: %d, Pressure: %d, Humidity: %d TemperatureFine: %d Voltage %d", (int)temperature, (int)pressure, (int)humidity, (int)temperatureFine, (int)voltage);

            return 0;
        }

        int getResultsJSON(char* buf, size_t bufSize) const override
        {
            return snprintf(buf, bufSize, 
                "{\"status\":\"ok\",\"timestamp\":%"PRIu32",\"temperature\":%"PRIi32",\"pressure\":%"PRIu32",\"humidity\":%"PRIu16",\"voltage\":%"PRIu16"}",
                actualCalculatedData.timestamp,
                actualCalculatedData.temperature,
                actualCalculatedData.pressure,
                actualCalculatedData.humidity,
                actualCalculatedData.voltage
            );
        }

        struct CalculatedData 
        {
            uint32_t temperature = -1;
            uint32_t pressure = -1;
            uint32_t humidity = -1;

            uint16_t voltage = -1;
        };


        

    private:
        
        struct CalculatedData2
        {
            uint32_t timestamp;

            int32_t temperature;
            uint32_t pressure;
            uint16_t humidity;
            uint16_t voltage;
        };
        CalculatedData2 actualCalculatedData;

        struct CalibrationData2
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
