#if defined(RP2040)
// <cmd 1 B> <data len 1 B> <data 0 B up to 62 B>
// cmd: CCCC CREE
// C -> command bit
// R -> reserve bit (0)
// E -> error response bit 
#include <stdint.h>

const uint8_t usbCommandMask = 0xf8;
enum class UsbCommand: uint8_t 
{
    AvailableData = 0x10,
    GetData = 0x20,
    GetSensorInfo = 0x21,     // <cmd 1 B> <data len 1 B> <Sensor identifier 1 B>
    GetDataFromSensor = 0x22, // <cmd 1 B> <data len 1 B> <Sensor identifier 1 B>
    GetPairedSensors,
    GetDataFromSensor,
    GetStatus,
    GetSensorStatus,
    InitializeSensor,
    SendRawDataToSensor = 0b11000000
};

const uint8_t usbErrorMask = 0x03;
enum class UsbErrorCode: uint8_t
{
    NoError = 0x00,
    BadCommandError = 0x01,
    LenghtError = 0x02,
    InternalCommandError = 0x03
};
#endif