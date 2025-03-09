#pragma once
#if defined(RP2040)
// <cmd 1 B> <data len 1 B> <data 0 B up to 62 B>
// cmd: EERC CCCC
// C -> command bit
// R -> reserved bit (0)
// E -> error response bit 
// Data are sent little endian
#include <stdint.h>

// Request  is Host   -->  Device
// Response is Device -->  Host 

const uint8_t usbCommandMask = 0x3f;
enum class UsbCommand: uint8_t  
{
    AvailableData = 0x10,
    GetData = 0x1f, // TODO remove
    GetKnownSensorsCount = 0x20,      //  Request data: null
                              //  Response data: <Sensors count 4 B>

    GetKnownSensors = 0x21,   //  Request data: null or <Page of sensors 1 B>
                              //  Response data: <Sensor identifier 4 B> (<Sensor identifier 4 B> ...)
                              // Error responses:
                              //  Response data: <Not known sensors 4 B>

    GetSensorInfo = 0x22,     //  Request data:  <Sensor identifier 4 B>
                              //  Response data: <Sensor identifier 4 B> <Sensor type 1 B> <Software ver. 4 B> <Hardware ver. 4 B>
                              // Error responses:
                              //  Response data: <Sensor not paired 4 B>
                              //  Response data: <Sensor out of range 4 B>
                              //  Response data: <Bad data lenght 4 B>
    
    GetSensorDataCount = 0x23, //  Request data:  <Sensor identifier 4 B>
                               //  Response data: <Sensor identifier 4 B> <Data count 4 B>

    GetSensorData = 0x24,     //  Request data:  <Sensor identifier 4 B>
                              //  Response data: <Raw data depends of sensor type, up to 62 B>
                              // Error responses:
                              //  Response data: <Sensor not paired 4 B>
                              //  ~~Response data: <Sensor out of range 4 B>~~
                              //  Response data: <Bad data lenght 4 B>
    GetStatus = 0x3c, // TODO change value
    GetSensorStatus = 0x3d, // TODO change value
    InitializeSensor = 0x3e, // TODO change value
    SendRawDataToSensor = 0x3f
};

const uint8_t usbErrorMask = 0xc0;
enum class UsbErrorCode: uint8_t
{
    NoError = 0x00,
    BadCommandError = 0x40,
    LenghtError = 0x80,
    InternalCommandError = 0xc0
};

enum class UsbCommandErrorCode: uint32_t
{
    NoError          = 0x00000000,
    BadDataLenght    = 0x00000001,
    FunctionNotSupported = 0x00000002,
    NotKnownSensors   = 0x01000000,
    SensorNotKnown  = 0x02000000,
    SensorOutOfRange = 0x02000001,
    SensorNoDataAvailable = 0x02000002
};
#endif