from .sensor_type import SensorType
from .sensor_measurement_type import SensorMeasurementType

def sensor_get_measurement_type_by_sensor_type(sensor_type: SensorType) -> list[SensorMeasurementType]:
    if sensor_type == SensorType.SimpleWeatherSensorBMP280:
        return [SensorMeasurementType.Temperature, SensorMeasurementType.Pressure]
    elif sensor_type == SensorType.SimpleWeatherSensorBME280:
        return [SensorMeasurementType.Temperature, SensorMeasurementType.Pressure, SensorMeasurementType.Humidity]
    elif sensor_type == SensorType.SimpleWeatherSensorBME680:
        return [SensorMeasurementType.Temperature, SensorMeasurementType.Pressure, SensorMeasurementType.Humidity]
    else:
        raise ValueError("Unknown sensor type")

