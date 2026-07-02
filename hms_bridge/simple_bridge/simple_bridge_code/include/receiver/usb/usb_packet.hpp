/// @file usb_packet.hpp
/// @brief Contain USB packet description and packet commands
/// @details 
///
/// Request  is Host   -->  Device
///
/// Response is Device -->  Host 
///
/// Device only answers the Host requests
///
/// USB packet structure:
/// <command byte 1 B> <data len 1 B> <data 0 B up to 62 B>
/// 
/// command byte: EERC CCCC
/// - C -> command bit
/// - R -> reserved bit (0)
/// - E -> error response bit 
///
/// @note Data are sent little endian
///
/// @author bravePeer
/// @date 17-08-2025

#pragma once
#include <stdint.h>

const uint8_t usbCommandMask = 0x3f;

/// @brief USB commands
enum class UsbCommand: uint8_t  
{
    /// @brief @todo Idk were is it needed
    AvailableData = 0x10,

    /// Not used?
    /// @todo Remove
    GetData = 0x1f,
    
    /// @brief Get Known Sensors Count
    ///
    /// @details Responses with known Sensors count
    ///
    /// Request data: null
    ///
    /// Response data: <Sensors count 4 B>
    ///
    GetKnownSensorsCount = 0x20,      

    /// @brief Get Known Sensors
    ///
    /// @details Responses with known Sensors identifiers (optional) starting with specific page
    ///
    /// Request data: null or <Page of sensors 1 B>
    ///
    /// Response data: <Sensor identifier 4 B> (<Sensor identifier 4 B> ...)
    ///
    /// Error responses:
    /// - @ref UsbCommandErrorCode::NotKnownSensors
    /// - @ref UsbCommandErrorCode::PageWithNoData
    GetKnownSensors = 0x21,   

    /// @brief Get Sensor Info
    /// 
    /// @note Radio address is not used for now
    ///
    /// @details Responses with sensor identifier, type of sensor and additional information
    ///
    /// Request data:  <Sensor identifier 4 B>
    ///
    /// Response data: <Sensor identifier 4 B> <Sensor type 4 B> <Software ver. 4 B> <Hardware ver. 4 B> 
    ///                <Radio address 5 B> <Init time 8 B>
    ///
    /// Error responses:
    /// - @ref UsbCommandErrorCode::BadDataLenght
    /// - @ref UsbCommandErrorCode::SensorNotKnown
    GetSensorInfo = 0x22,   
    
    /// @brief Get Sensor Calibration Data
    ///
    /// @details Responses with identifier and calibration data dependent on sensor type
    ///
    /// Request data:  <Sensor identifier 4 B> <[optional] page id 1 B>
    ///
    /// Response data: <Sensor identifier 4 B> <Calibration data depends of sensor type, up to 62 B>
    ///
    /// Error responses:
    /// - @ref UsbCommandErrorCode::BadDataLenght
    /// - @ref UsbCommandErrorCode::PageWithNoData
    /// - @ref UsbCommandErrorCode::SensorNotKnown
    /// - @ref UsbCommandErrorCode::SensorCalibrationDataNotKnown
    /// @todo mod GetSensorCalibData
    GetSensorCalibData = 0x23, 
    
    /// @brief Get Sensor Data Count
    ///
    /// @details Responses with count of available data
    ///
    /// Request data:  <Sensor identifier 4 B>
    ///
    /// Response data: <Sensor identifier 4 B> <Data count 4 B>
    /// Error responses:
    /// - @ref UsbCommandErrorCode::BadDataLenght
    GetSensorDataCount = 0x24, 

    /// @brief Get Sensor Data
    ///
    /// @details Responses with raw sensor data 
    ///
    /// Request data:  <Sensor identifier 4 B>
    ///
    /// Response data: <Raw data depends of sensor type, up to 62 B>
    ///
    /// Error responses:
    /// - @ref UsbCommandErrorCode::BadDataLenght
    /// - @ref UsbCommandErrorCode::SensorNotKnown
    GetSensorData = 0x25,     

                              
    /// @brief Get Sensor Last Data
    ///
    /// @details This command is used to get last data from sensor and not remove it from buffer.
    /// Is much slower than @ref GetSensorData.
    ///
    /// Request data:  <Sensor identifier 4 B>
    ///
    /// Response data: <Raw data depends of sensor type, up to 62 B>
    ///
    /// Error responses:
    /// - @ref UsbCommandErrorCode::BadDataLenght
    /// - @ref UsbCommandErrorCode::SensorNotKnown
    GetSensorLastData = 0x26, 

    /// @brief @todo Implement Get Sensor Status
    GetSensorStatus = 0x27,

    GetStatus = 0x3c, // TODO change value
    InitializeSensor = 0x3e, // TODO change value
    SendRawDataToSensor = 0x3f
};

const uint8_t usbCommandErrorMask = 0xc0;

/// @brief Command byte error flags
enum class UsbErrorCode: uint8_t
{
    NoError = 0x00,
    BadCommandError = 0x40,
    LenghtError = 0x80,
    InternalCommandError = 0xc0
};


/// @brief Possible error response codes
/// @details 
/// Internal command error code
///
/// Codes categories where x is any value:
/// - 0x00xxxxxx Generic errors
/// - 0x01xxxxxx Sensor communication errors
/// - 0x02xxxxxx Sensor specific errors
enum class UsbCommandErrorCode: uint32_t
{
    NoError                       = 0x00000000,
    BadDataLenght                 = 0x00000001,
    FunctionNotSupported          = 0x00000002,
    PageWithNoData                = 0x00000003,
    
    NotKnownSensors               = 0x01000000,

    SensorNotKnown                = 0x02000000,
    SensorOutOfRange              = 0x02000001,
    SensorNoDataAvailable         = 0x02000002,
    SensorCalibrationDataNotKnown = 0x02000003,
};

constexpr uint8_t getRawUsbCommand(UsbCommand command)
{
    return static_cast<uint8_t>(command);
}

constexpr UsbCommand getUsbCommand(uint8_t data)
{
    return static_cast<UsbCommand>(data & usbCommandMask);
}

constexpr uint8_t getUsbResponseCommand(UsbCommand usbCommand, UsbErrorCode usbErrorCode)
{
    return static_cast<uint8_t>(usbCommand) | static_cast<uint8_t>(usbErrorCode);
}

// struct UsbPacketRaw
// {
//     uint8_t headerRaw;
//     uint8_t size;
//     uint8_t data[]
// };
