#pragma once
#include <stdint.h>
#include "spi.hpp"
#include <stddef.h>

extern uint8_t transmitSpiBme280(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len);

namespace bme280
{
    enum class MemoryMap: uint8_t
    {
        HumLsb = 0xFE,
        HumMsb = 0xFD,
        TempXlsb = 0xFC,
        TempLsb = 0xFB,
        TempMsb = 0xFA,
        PressXlsb = 0xF9,
        PressLsb = 0xF8,
        PressMsb = 0xF7,
        Config = 0xF5,
        CtrlMeas = 0xF4,
        Status = 0xF3,
        CtrlHum = 0xF2,
        Reset = 0xE0,
        Id = 0xD0,
        CalibT1LSB = 0x88, // Begin of calibration data
        CalibT1MSB = 0x89, 
        CalibT2LSB = 0x8A, 
        CalibT2MSB = 0x8B, 
        CalibT3LSB = 0x8C, 
        CalibT3MSB = 0x8D, 
        CalibP1LSB = 0x8E, 
        CalibP1MSB = 0x8F, 
        CalibP2LSB = 0x90, 
        CalibP2MSB = 0x91, 
        CalibP3LSB = 0x92, 
        CalibP3MSB = 0x93, 
        CalibP4LSB = 0x94, 
        CalibP4MSB = 0x95, 
        CalibP5LSB = 0x96, 
        CalibP5MSB = 0x97, 
        CalibP6LSB = 0x98, 
        CalibP6MSB = 0x99, 
        CalibP7LSB = 0x9A, 
        CalibP7MSB = 0x9B, 
        CalibP8LSB = 0x9C, 
        CalibP8MSB = 0x9D, 
        CalibP9LSB = 0x9E, 
        CalibP9MSB = 0x9F, 
        CalibReservedLSB = 0xA0,
        CalibReservedMSB = 0xA1,
    };

    enum class ControlOversamplingTemperature
    {
        Skipped         = 0b000,
        OversamplingX1  = 0b001,
        OversamplingX2  = 0b010,
        OversamplingX4  = 0b011,
        OversamplingX8  = 0b100,
        OversamplingX16 = 0b101,
        // OversamplingX16 = 0b110,
        // OversamplingX16 = 0b111,
    };

    enum class ControlOversamplingPressure
    {
        Skipped         = 0b000,
        OversamplingX1  = 0b001,
        OversamplingX2  = 0b010,
        OversamplingX4  = 0b011,
        OversamplingX8  = 0b100,
        OversamplingX16 = 0b101,
        // OversamplingX16 = 0b110,
        // OversamplingX16 = 0b111,
    };
    
    enum class ControlMode
    {
        Sleep  = 0b00,
        Forced = 0b01,
        // Forced = 0b10,
        Normal = 0b11
    };

    #pragma pack(push, 1)
    struct ControlMeasurements
    {
        ControlOversamplingTemperature ctrlTemperature : 3;
        ControlOversamplingPressure ctrlPressure : 3;
        ControlMode mode : 2;
    };
    #pragma pack(pop)
    ControlMeasurements lastControlMeasurements;

    constexpr uint8_t measuringMask = 0x08;
    constexpr uint8_t imUpdateMask = 0x01;
    constexpr uint8_t busyMask = measuringMask | imUpdateMask;

    constexpr uint8_t commandWriteBytes(MemoryMap address)
    {
        return static_cast<uint8_t>(address) & 0x7f;
    }

    constexpr uint8_t commandReadBytes(MemoryMap address)
    {
        return static_cast<uint8_t>(address) | 0x80;
    }

    constexpr uint8_t toByte(const ControlMeasurements& ctrlMeas)
    {
        return static_cast<uint8_t>(
            static_cast<uint8_t>(ctrlMeas.ctrlTemperature) << 5 |
            static_cast<uint8_t>(ctrlMeas.ctrlPressure) << 2 | 
            static_cast<uint8_t>(ctrlMeas.mode)
        );
    }

    /// @brief 
    /// @param idBuffer pointer to one byte buffer where id will be stored
    inline void readId(uint8_t* idBuffer)
    {
        transmitSpiBme280(idBuffer, idBuffer, commandReadBytes(MemoryMap::Id), 1);
    }

    inline void softReset()
    {
        uint8_t data = 0xe0;
        transmitSpiBme280(&data, &data, commandReadBytes(MemoryMap::Reset), 1);
    }

    inline void writeControlMeasurements(const ControlMeasurements ctrlMeas)
    {
        uint8_t data = toByte(ctrlMeas);
        lastControlMeasurements = ctrlMeas;
        transmitSpiBme280(&data, &data, commandWriteBytes(MemoryMap::CtrlMeas), 1);
    }

    /// @brief Reads the pressure and temperature data from the BME280 sensor.
    /// @param data Pointer to a buffer where the sensor data will be stored. 
    ///             The buffer must be at least 6 bytes long.
    /// @tparam N Size of the data buffer, must be at least 6 bytes.
    template <size_t N>
    void readAllDataBmp280(uint8_t (&data)[N])
    {
        static_assert(N >= 6, "Buffer size must be at least 6 bytes to read pressure and temperature data");
        transmitSpiBme280(data, data, commandReadBytes(MemoryMap::PressMsb), 6);
    }

    inline uint8_t isMeasuring()
    {
        uint8_t data = 0;
        transmitSpiBme280(&data, &data, commandWriteBytes(MemoryMap::Status), 1);
        return data & measuringMask;
    }

    inline uint8_t isImageUpdating()
    {
        uint8_t data = 0;
        transmitSpiBme280(&data, &data, commandWriteBytes(MemoryMap::Status), 1);
        return data & imUpdateMask;
    }

    inline uint8_t isBusy()
    {
        uint8_t data = 0;
        transmitSpiBme280(&data, &data, commandWriteBytes(MemoryMap::Status), 1);
        return data & busyMask;
    }

    inline void startForceMeasurement()
    {
        lastControlMeasurements.mode = ControlMode::Forced;
        uint8_t data = toByte(lastControlMeasurements);
        transmitSpiBme280(&data, &data, commandWriteBytes(MemoryMap::CtrlMeas), 1);
    }

    /// @brief Reads the calibration data from the BME280 sensor.
    /// @param data Pointer to a buffer where the calibration data will be stored. 
    ///             The buffer must be at least 26 bytes long.
    /// @tparam N Size of the data buffer, must be at least 6 bytes.
    template <size_t N>
    void readCalibrationData(uint8_t (&data)[N])
    {
        static_assert(N >= 26, "Buffer size must be at least 6 bytes to read calibration data");
        transmitSpiBme280(data, data, commandReadBytes(MemoryMap::CalibT1LSB), 26);
    }
}
