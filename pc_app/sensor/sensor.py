from .sensor_types import SensorType
from .sensor_error import SensorExceptionGeneric, SensorExceptionCalibrationData

class Sensor:
    def __init__(self):
        self.sensor_type = SensorType.Unknown
        self.identifier = -1
        self.software_version = -1
        self.hardware_version = -1
        self.address = []
        self.initialization_time = -1
        self.last_recv_data_time = -1
        self.calibration_data = []
        self.is_calibration_data_known = False
        self.data = []

        self._IS_CALIBRATION_DATA_NEEDED = True

    def __str__(self):
        ret = f"Sensor identifier: 0x{self.identifier:08X}\n"
        ret += f"Sensor type: 0x{self.sensor_type:02X}\n"
        ret += f"Software version: 0x{self.software_version:04X}\n"
        ret += f"Hardware version: 0x{self.hardware_version:04X}\n"
        ret += f"Address: {self.address}\n"
        ret += f"Initialization time: {self.initialization_time}\n"
        ret += f"Last received data time: {self.last_recv_data_time}\n"
        ret += f"Is calibration data known: {self.is_calibration_data_known}\n"
        
        if self.is_calibration_data_known:
            ret += f"Calibration data: {self.calibration_data}"

        return ret

    def set_calibration_data(self, calibration_data):
        self.calibration_data = calibration_data
        self.is_calibration_data_known = True

    def set_sensor_info(self, raw_data):
        self.identifier = int.from_bytes(raw_data[0:4], "little")
        self.sensor_type = raw_data[4]
        self.software_version = int.from_bytes(raw_data[5:9], "little")
        self.hardware_version = int.from_bytes(raw_data[9:13], "little")
        self.address = raw_data[13:18]
        self.initialization_time = int.from_bytes(raw_data[18:22], "little")
        self.last_recv_data_time = int.from_bytes(raw_data[22:26], "little")

    def enqueue_data(self, data):
        self.data.append(data)

    def dequeue_data(self):
        return self.data.pop(0)

    def _check_calibration_data(self):
        if self._IS_CALIBRATION_DATA_NEEDED == False:
            return
        if self.is_calibration_data_known == True:
            return
        raise SensorExceptionCalibrationData()


class SimpleWeatherStation(Sensor):
    def __init__(self):
        super().__init__()
        self.sensor_type = SensorType.SimpleWheatherStation
        self._IS_CALIBRATION_DATA_NEEDED = True

    def get_temperature(self)->float:
        self._check_calibration_data()

    def get_humidity(self)->float:
        self._check_calibration_data()

    def get_pressure(self)->float:
        self._check_calibration_data()