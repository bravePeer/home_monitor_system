from .sensor_types import SensorType
from .sensor_error import SensorExceptionGeneric, SensorExceptionCalibrationData
from struct import unpack

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

        self.verbose = False

    def __str__(self):
        ret = f"Sensor class type: {self.__class__.__name__}\n"
        ret += f"Sensor identifier: 0x{self.identifier:08X}\n"
        ret += f"Sensor type: {self.sensor_type}\n"
        ret += f"Sensor type value: 0x{int(self.sensor_type):02X}\n"
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
        self.sensor_type = SensorType(raw_data[4])
        self.software_version = int.from_bytes(raw_data[5:9], "little")
        self.hardware_version = int.from_bytes(raw_data[9:13], "little")
        self.address = raw_data[13:18]
        self.initialization_time = int.from_bytes(raw_data[18:22], "little")
        self.last_recv_data_time = int.from_bytes(raw_data[22:26], "little")

    def enqueue_data(self, data):
        self.data.append(data)

    def dequeue_data(self):
        return self.data.pop(0)

    def get_available_functions(self) -> list[str]:
        return [func for func in dir(self) if callable(getattr(self, func)) and func.startswith("sfunc")] # Sensor functions


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

    # Calibration data BMP280
    # dig_t1 = self.calibration_data[0] | (self.calibration_data[1] << 8)
    # dig_t2 = self.calibration_data[2] | (self.calibration_data[3] << 8)
    # dig_t3 = self.calibration_data[4] | (self.calibration_data[5] << 8)
    # dig_p1 = self.calibration_data[6] | (self.calibration_data[7] << 8)
    # dig_p2 = self.calibration_data[8] | (self.calibration_data[9] << 8)
    # dig_p3 = self.calibration_data[10] | (self.calibration_data[11] << 8)
    # dig_p4 = self.calibration_data[12] | (self.calibration_data[13] << 8)
    # dig_p5 = self.calibration_data[14] | (self.calibration_data[15] << 8)
    # dig_p6 = self.calibration_data[16] | (self.calibration_data[17] << 8)
    # dig_p7 = self.calibration_data[18] | (self.calibration_data[19] << 8)
    # dig_p8 = self.calibration_data[20] | (self.calibration_data[21] << 8)
    # dig_p9 = self.calibration_data[22] | (self.calibration_data[23] << 8)
    # reserved = self.calibration_data[24] | (self.calibration_data[25] << 8)

    def __calculate_temperature_fine(self):
        # TODO select calculation data
        adc_t = int.from_bytes(self.data[0][14:18], "little")
        adc_t = adc_t >> 4
        
        tmp_calib = bytes(self.calibration_data)
        tmp_calib = unpack('<Hhh', tmp_calib[0:6])
        dig_t1 = tmp_calib[0]
        dig_t2 = tmp_calib[1]
        dig_t3 = tmp_calib[2]
        if self.verbose:
            print(f"adc_t: {adc_t}")
            print(f"dig_t1: {dig_t1}")
            print(f"dig_t2: {dig_t2}")
            print(f"dig_t3: {dig_t3}")
        tmp1 = (((adc_t >> 3) - (dig_t1 << 1)) * dig_t2) >> 11
        tmp2 = (((((adc_t >> 4) - dig_t1) * ((adc_t >> 4) - dig_t1)) >> 12) * dig_t3) >> 14
        fine_t = tmp1 + tmp2
        return fine_t

    
    def sfunc_get_temperature(self) -> float:
        """Calculate temperature in Celsius."""
        self._check_calibration_data()
        fine_t = self.__calculate_temperature_fine()
        return ((fine_t * 5 + 128) >> 8) / 100.0
         
    def sfunc_get_humidity(self)->float:
        return None

    def sfunc_get_pressure(self)->float:
        """Calculate pressure in Pa."""
        self._check_calibration_data()
        fine_t = self.__calculate_temperature_fine()

        adc_p = int.from_bytes(self.data[0][18:22], "little")
        adc_p = adc_p >> 4

        tmp_calib = bytes(self.calibration_data)
        tmp_calib = unpack('<Hhhhhhhhh', tmp_calib[6:24])
        dig_p1 = tmp_calib[0] 
        dig_p2 = tmp_calib[1]
        dig_p3 = tmp_calib[2]
        dig_p4 = tmp_calib[3]
        dig_p5 = tmp_calib[4]
        dig_p6 = tmp_calib[5]
        dig_p7 = tmp_calib[6]
        dig_p8 = tmp_calib[7]
        dig_p9 = tmp_calib[8]
        if self.verbose:
            print(f"fine_t: {fine_t}")
            print(f"adc_p: {adc_p}")
            print(f"dig_p1: {dig_p1}")
            print(f"dig_p2: {dig_p2}")
            print(f"dig_p3: {dig_p3}")
            print(f"dig_p4: {dig_p4}")
            print(f"dig_p5: {dig_p5}")
            print(f"dig_p6: {dig_p6}")
            print(f"dig_p7: {dig_p7}")
            print(f"dig_p8: {dig_p8}")
            print(f"dig_p9: {dig_p9}")

        var1 = fine_t - 128000
        if self.verbose:
            print(f"var1: {var1}")
        var2 = var1 * var1 * dig_p6
        if self.verbose:
            print(f"var2: {var2}")
        var2 = var2 + ((var1 * dig_p5) << 17)
        if self.verbose:
            print(f"var2: {var2}")
        var2 = var2 + ((dig_p4) << 35)
        if self.verbose:
            print(f"var2: {var2}")
        var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12)
        if self.verbose:
            print(f"var1: {var1}")
        var1 = ((1 << 47) + var1) * (dig_p1) >> 33
        if self.verbose:
            print(f"var1: {var1}")
        if var1 == 0:
            print("Pressure calculation error")
            return 0
        p = 1048576 - adc_p
        if self.verbose:
            print(f"p: {p}")
        p = (((p << 31) - var2) * 3125) // var1
        if self.verbose:
            print(f"p: {p}")
        var1 = (dig_p9 * (p >> 13) * (p >> 13)) >> 25
        if self.verbose:
            print(f"var1: {var1}")
        var2 = (dig_p8 * p) >> 19
        if self.verbose:
            print(f"var2: {var2}")
        p = ((p + var1 + var2) >> 8) + (dig_p7 << 4)
        if self.verbose:
            print(f"p: {p}")
        
        return float(p) / 256.0

    def sfunc_get_battery_voltage(self)->float:
        pass