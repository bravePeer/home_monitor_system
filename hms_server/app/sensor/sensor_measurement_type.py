from enum import StrEnum
from ..database.sensor_data_query import sensor_data_battery_voltage_create, sensor_data_humidity_create, sensor_data_pressure_create, sensor_data_temperature_create
from typing import Any

class SensorMeasurementType(StrEnum):
    Temperature = "temperature_celsius"
    Humidity = "humidity_percent"
    Pressure = "pressure_pa"
    BatteryVoltage = "battery_voltage"


SensorMeasurementTypeToCreateFunctionMap: dict[SensorMeasurementType, Any] = {
    SensorMeasurementType.Temperature: sensor_data_temperature_create,
    SensorMeasurementType.Humidity: sensor_data_humidity_create,
    SensorMeasurementType.Pressure: sensor_data_pressure_create,
    SensorMeasurementType.BatteryVoltage: sensor_data_battery_voltage_create,
}
