from enum import IntEnum

class SensorType(IntEnum):
    Test = 0
    SimpleWeatherSensorGeneric = 0x00010000
    SimpleWeatherSensorBMP280  = 0x00010058
    SimpleWeatherSensorBME280  = 0x00010060
    SimpleWeatherSensorBME680  = 0x00010061
    Unknown = 0xffffffff

