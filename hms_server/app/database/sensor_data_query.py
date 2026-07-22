from .database import SessionDep
from .database_models.sensor_models import SensorDataBatteryVoltageModel, SensorDataHumidityModel, SensorDataPressureModel, SensorDataTemperatureModel

def sensor_data_temperature_create(sensor_id: int, temperature_raw: int, session: SessionDep) -> SensorDataTemperatureModel:
    new_data = SensorDataTemperatureModel(sensor_id=sensor_id, temperature_raw=temperature_raw)
    session.add(new_data)
    session.commit()
    session.refresh(new_data)
    return new_data

def sensor_data_humidity_create(sensor_id: int, humidity_raw: int, session: SessionDep) -> SensorDataHumidityModel:
    new_data = SensorDataHumidityModel(sensor_id=sensor_id, humidity_raw=humidity_raw)
    session.add(new_data)
    session.commit()
    session.refresh(new_data)
    return new_data

def sensor_data_pressure_create(sensor_id: int, pressure_raw: int, session: SessionDep) -> SensorDataPressureModel:
    new_data = SensorDataPressureModel(sensor_id=sensor_id, pressure_raw=pressure_raw)
    session.add(new_data)
    session.commit()
    session.refresh(new_data)
    return new_data

def sensor_data_battery_voltage_create(sensor_id: int, battery_voltage_raw: int, session: SessionDep) -> SensorDataBatteryVoltageModel:
    new_data = SensorDataBatteryVoltageModel(sensor_id=sensor_id, voltage_raw=battery_voltage_raw)
    session.add(new_data)
    session.commit()
    session.refresh(new_data)
    return new_data
