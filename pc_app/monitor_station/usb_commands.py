
from enum import Enum

# <cmd 1 B> <data len 1 B> <data 0 B up to 62 B>
# cmd: EERC CCCC
# C -> command bit
# R -> reserved bit (0)
# E -> error response bit
# Data are sent little endian
# Request  is Host   -->  Device
# Response is Device -->  Host
usbCommandMask = 0x3f
#                            //  Response data: <Sensors count 4 B>
#                            //  Response data: <Sensor identifier 4 B> (<Sensor identifier 4 B> ...)
#                            // Error responses:
#                            //  Response data: <Not known sensors 4 B>
#                            //  Response data: <Sensor identifier 4 B> <Sensor type 1 B> <Software ver. 4 B> <Hardware ver. 4 B>
#                            // Error responses:
#                            //  Response data: <Sensor not paired 4 B>
#                            //  Response data: <Sensor out of range 4 B>
#                            //  Response data: <Bad data lenght 4 B>
#                             //  Response data: <Sensor identifier 4 B> <Data count 4 B>
#                            //  Response data: <Raw data depends of sensor type, up to 62 B>
#                            // Error responses:
#                            //  Response data: <Sensor not paired 4 B>
#                            //  ~~Response data: <Sensor out of range 4 B>~~
#                            //  Response data: <Bad data lenght 4 B>
class UsbCommand(Enum):
    AvailableData = 0x10
    GetData = 0x1f
    GetKnownSensorsCount = 0x20
    GetKnownSensors = 0x21
    GetSensorInfo = 0x22
    GetSensorDataCount = 0x23
    GetSensorData = 0x24
    GetStatus = 0x3c
    GetSensorStatus = 0x3d
    InitializeSensor = 0x3e
    SendRawDataToSensor = 0x3f


usbErrorMask = 0xc0
class UsbErrorCode(Enum):
    NoError = 0x00
    BadCommandError = 0x40
    LenghtError = 0x80
    InternalCommandError = 0xc0


class UsbCommandErrorCode(Enum):
    NoError          = 0x00000000
    BadDataLenght    = 0x00000001
    FunctionNotSupported = 0x00000002
    NotKnownSensors   = 0x01000000
    SensorNotKnown  = 0x02000000
    SensorOutOfRange = 0x02000001
    SensorNoDataAvailable = 0x02000002

