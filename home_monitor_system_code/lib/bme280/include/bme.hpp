#pragma once
#include <stdint.h>
#include <assert.h>
#include "traits.hpp"
#include "bme_common.hpp"
#include "bme280.hpp"
#include "bme680.hpp"
#include "bmp280.hpp"

namespace bme
{
    enum class BmeType: uint8_t
    {
        BadType = 0x00,
        BME280,
        BMP280,
        BME680
    };

    template<typename T>
    constexpr typename traits::enable_if<
        traits::is_same<T, bme280::MemoryMap>::value || 
        traits::is_same<T, bmp280::MemoryMap>::value || 
        traits::is_same<T, bme680::MemoryMap>::value,
        uint8_t
    >::type
    commandWriteBytes(T address)
    {
        return static_cast<uint8_t>(address) & 0x7f;
    }

    /// @brief 
    /// @tparam T awd
    /// @param address 
    /// @return 
    template<typename T>
    constexpr typename traits::enable_if<
        traits::is_same<T, bme280::MemoryMap>::value || 
        traits::is_same<T, bmp280::MemoryMap>::value || 
        traits::is_same<T, bme680::MemoryMap>::value,
        uint8_t
    >::type
    commandReadBytes(T address)
    {
        return static_cast<uint8_t>(address) | 0x80;
    }


    /// @brief Checks id of BME chip
    /// @param buffer Min len of buffer is 1 byte. In buffer[0] will be store BME chip ID
    /// @return BME type if not valid return 0 (BadType)
    inline BmeType isIdValid(uint8_t* buffer)
    {
        static_assert(commandReadBytes(bme280::MemoryMap::Id) == commandReadBytes(bme680::MemoryMap::Id_page0));
        transmitSpiBme280(buffer, buffer, commandReadBytes(bme280::MemoryMap::Id), 1);

        switch (buffer[0])
        {
        case bme280::idValue:
            return BmeType::BME280;
        case bme680::idValue:
            return BmeType::BME680;
        default:
            return BmeType::BadType;
        }
    }

    /// @brief Checks id of BME chip
    /// @return BME type if not valid return 0 (BadType)
    inline BmeType isIdValid()
    {
        static_assert(commandReadBytes(bme280::MemoryMap::Id) == commandReadBytes(bme680::MemoryMap::Id_page0));
        uint8_t buffer[1];
        transmitSpiBme280(buffer, buffer, commandReadBytes(bme280::MemoryMap::Id), 1);

        switch (buffer[0])
        {
        case bme280::idValue:
            return BmeType::BME280;
        case bme680::idValue:
            return BmeType::BME680;
        default:
            return BmeType::BadType;
        }
    }


    template<typename T>
    inline typename traits::enable_if<
        traits::is_same<T, bme280::MemoryMap>::value || 
        traits::is_same<T, bmp280::MemoryMap>::value || 
        traits::is_same<T, bme680::MemoryMap>::value,
        void
    >::type
    writeReg(uint8_t* data, T reg)
    {
        transmitSpiBme280(data, data, commandWriteBytes(reg), 1);
    }

    template<typename T>
    inline typename traits::enable_if<
        traits::is_same<T, bme280::MemoryMap>::value || 
        traits::is_same<T, bmp280::MemoryMap>::value || 
        traits::is_same<T, bme680::MemoryMap>::value,
        void
    >::type
    readReg(uint8_t* data, T reg)
    {
        transmitSpiBme280(data, data, commandReadBytes(reg), 1);
    }

    template<BmeType bmeType>
    struct BmeSelector;

    template<>
    struct BmeSelector<BmeType::BME280>
    {
        static int8_t isValid()
        {
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandReadBytes(bme280::MemoryMap::Id), 1);

            if(data == bme280::idValue)
                return 1;
            
            return 0;            
        }

        static int8_t isValid(uint8_t* data)
        {
            transmitSpiBme280(data, data, commandReadBytes(bme280::MemoryMap::Id), 1);

            if(data[0] == bme280::idValue)
                return 1;
            
            return 0;            
        }

        static uint8_t isBusy() 
        { 
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandWriteBytes(bme280::MemoryMap::Status), 1);
            return data & bme280::busyMask;
        }

        static void softReset() 
        {
            uint8_t data = bme280::resetValue;
            transmitSpiBme280(&data, &data, commandWriteBytes(bme280::MemoryMap::Reset), 1);
        }

        template <size_t N>
        static void readAllData(uint8_t (&data)[N])
        {
            static_assert(N >= 8, "Buffer size must be at least 8 bytes to read pressure, temperature and humidity data");
            transmitSpiBme280(data, data, commandReadBytes(bme280::MemoryMap::PressMsb), 8);
        }

        /// @brief Read calibration parameters 0x88 - 0xA1
        /// @tparam N 
        /// @param data 
        template <size_t N>
        static void readCalibrationData0(uint8_t (&data)[N])
        {
            static_assert(N >= 26, "Buffer size must be at least 6 bytes to read pressure and temperature data");
            transmitSpiBme280(data, data, commandReadBytes(bme280::MemoryMap::CalibT1LSB), 26);
        }

        /// @brief Read calibration parameters 0xE1 - 0xF0
        /// @tparam N 
        /// @param data 
        template <size_t N>
        static void readCalibrationData1(uint8_t (&data)[N])
        {
            static_assert(N >= 6, "Buffer size must be at least 6 bytes to read pressure and temperature data");
            transmitSpiBme280(data, data, commandReadBytes(bme280::MemoryMap::CalibH2LSB), 15);
        }
    
        static void startForcedMeasurement()
        {
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandReadBytes(bme280::MemoryMap::Status), 1);
            data = (data & (~0x03)) | static_cast<uint8_t>(bme::ControlMode::Forced);
            transmitSpiBme280(&data, &data, commandWriteBytes(bme280::MemoryMap::Status), 1);
        }
    };
    
    template<>
    struct BmeSelector<BmeType::BMP280>
    {
        static int8_t isValid()
        {
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandReadBytes(bmp280::MemoryMap::Id), 1);

            if(data == bmp280::idValue)
                return 1;
            
            return 0;            
        }

        static int8_t isValid(uint8_t* data)
        {
            transmitSpiBme280(data, data, commandReadBytes(bmp280::MemoryMap::Id), 1);

            if(data[0] == bmp280::idValue)
                return 1;
            
            return 0;            
        }
    
        static uint8_t isBusy() 
        { 
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandReadBytes(bmp280::MemoryMap::Status), 1);
            return data & bmp280::busyMask;
        }

        static void softReset() 
        {
            uint8_t data = bmp280::resetValue;
            transmitSpiBme280(&data, &data, commandWriteBytes(bmp280::MemoryMap::Reset), 1);
        }

        template <size_t N>
        static void readAllData(uint8_t (&data)[N])
        {
            static_assert(N >= 6, "Buffer size must be at least 6 bytes to read pressure and temperature data");
            transmitSpiBme280(data, data, commandReadBytes(bmp280::MemoryMap::PressMsb), 6);
        }

        /// @brief Read calibration parameters 0x88 - 0xA1
        /// @tparam N 
        /// @param data 
        template <size_t N>
        static void readCalibrationData0(uint8_t (&data)[N])
        {
            static_assert(N >= 26, "Buffer size must be at least 6 bytes to read pressure and temperature data");
            transmitSpiBme280(data, data, commandReadBytes(bmp280::MemoryMap::CalibT1LSB), 26);
        }

        /// @brief Not used
        template <size_t N>
        static void readCalibrationData1(uint8_t (&data)[N])
        {
            // Nothing to do
        }
    
        static void startForcedMeasurement()
        {
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandReadBytes(bmp280::MemoryMap::CtrlMeas), 1);
            data |= static_cast<uint8_t>(bme::ControlMode::Forced);
            transmitSpiBme280(&data, &data, commandWriteBytes(bmp280::MemoryMap::CtrlMeas), 1);
        }
    
    };
    

    template<>
    struct BmeSelector<BmeType::BME680> 
    {
        static int8_t isValid()
        {
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandReadBytes(bme680::MemoryMap::Id_page0), 1);

            if(data == bme680::idValue)
                return 1;
            
            return 0;            
        }

        static int8_t isValid(uint8_t* data)
        {
            transmitSpiBme280(data, data, commandReadBytes(bme680::MemoryMap::Id_page0), 1);

            if(data[0] == bme680::idValue)
                return 1;
            
            return 0;            
        }

        static uint8_t isBusy() 
        { 
            uint8_t data = 0;
            transmitSpiBme280(&data, &data, commandWriteBytes(bme680::MemoryMap::Status), 1);
            return data & bme680::busyMask;
        }
        
        static void softReset() 
        {
            uint8_t data = bme680::resetValue;
            transmitSpiBme280(&data, &data, commandWriteBytes(bme680::MemoryMap::Reset_page0), 1);
        }

        template <size_t N>
        static void readAllData(uint8_t (&data)[N])
        {
            static_assert(N >= 10, "Buffer size must be at least 10 bytes to read TODO");
            transmitSpiBme280(data, data, commandReadBytes(bme680::MemoryMap::PressMSB), 6);
        }

        template <size_t N>
        static void readCalibT(uint8_t (&data)[N])
        {
            static_assert(N >= 5, "Buffer size must be at least 5 bytes to read temperature calibration parameters");
            transmitSpiBme280(data, data, commandReadBytes(bme680::MemoryMap::ParT1LSB), 2);
            transmitSpiBme280(data, data, commandReadBytes(bme680::MemoryMap::ParT2LSB), 3);
        }

        template <size_t N>
        static void readCalibP(uint8_t (&data)[N])
        {

        }


        template <size_t N>
        static void readCalibH(uint8_t (&data)[N])
        {

        }

        /// @brief Read calibration parameters for Temperature and Humidity
        /// @tparam N 
        /// @param data 
        template <size_t N>
        static void readCalibTH(uint8_t (&data)[N])
        {

        }
    };

}