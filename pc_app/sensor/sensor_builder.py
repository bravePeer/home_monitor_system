from .sensor import *
from .sensor_types import *
from .sensor_error import *

class SensorBuilderMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class SensorBuilder(metaclass=SensorBuilderMeta):
    def build_from_bytes(self, raw_data: bytes) -> Sensor:
        sensor = Sensor()
        sensor.set_sensor_info(raw_data)
        if sensor.sensor_type == SensorType.Test:
            raise SensorExceptionGeneric("Test sensor type is not supported!")
        elif sensor.sensor_type == SensorType.SimpleWheatherStation:
            sensor = SimpleWeatherStation()
            sensor.set_sensor_info(raw_data)
        elif sensor.sensor_type == SensorType.Unknown:
            raise SensorExceptionGeneric("Unknown sensor type")

        return sensor