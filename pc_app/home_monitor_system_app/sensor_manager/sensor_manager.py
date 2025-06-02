from sensor.sensor import Sensor
from sensor.sensor_builder import SensorBuilder
from monitor_station.monitor_station_communicator import MonitorStationCommunicator
from logger import Logger

class SensorManagerMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class SensorManager(metaclass=SensorManagerMeta):
    def __init__(self):
        self.sensors: list[Sensor] = []

    def get_not_known_sensors_identifiers(self) -> list[int]:
        
        count = MonitorStationCommunicator().read_sensors_count()
        if count == 0:
            return []
        
        ids = MonitorStationCommunicator().read_sensors_identifiers()
        for sensor in self.sensors:
            for id in ids[:]:
                if sensor.identifier == id:
                    ids.remove(id)
        Logger().info(f"Not known modules: {ids}", self.__class__.__name__)
        return ids

    def get_new_sensors(self, ids: list[int]):
        for id in ids:
            sensor_raw_info = MonitorStationCommunicator().read_sensor_info_raw(id)
            new_sensor = SensorBuilder().build_from_bytes(sensor_raw_info)
            if new_sensor._IS_CALIBRATION_DATA_NEEDED:
                new_sensor_calibration = MonitorStationCommunicator().read_calibration_data(id)
                new_sensor.set_calibration_data(new_sensor_calibration)
            self.sensors.append(new_sensor)
            Logger().info(f"Added new sensor: {id}", self.__class__.__name__)

    def update_sensor_data(self):
        for sensor in self.sensors:
            availabe_data = MonitorStationCommunicator().read_sensor_available_data_count(sensor.identifier)
            datas = MonitorStationCommunicator().read_sensor_available_data(sensor.identifier, availabe_data)
            for data in datas:
                sensor.enqueue_data(data)
                Logger().info(f"To sensor: {sensor.identifier}, added new data: {data}", self.__class__.__name__)
            