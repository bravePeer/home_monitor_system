from logger import logger
from .monitor_station_device import MonitorStationDevice
from .usb_commands import UsbCommand
from .utils import int_to_bytes


class MonitorStationCommunicator:
    # def __init__():
    #     pass

    def check_device(self):
        pass

    def read_sensors_count(self) -> int:
        device = MonitorStationDevice()
        packet = bytearray()
        packet.append(UsbCommand.GetKnownSensorsCount.value)
        packet.append(0)
        recv = device.write_read(packet)
        sensors_count = int.from_bytes(recv[2:6], "little")
        logger.info(f"Count of known sensors: {sensors_count}", self.__class__.__name__)
        return sensors_count

    def read_sensors_identifiers(self) -> list[int]:
        packet = bytearray()
        packet.append(UsbCommand.GetKnownSensors.value)
        packet.append(0)

        recv = MonitorStationDevice().write_read(packet)

        # TODO Read sensors identifiers from other pages
        sensors_identifiers = []
        for i in range(2, len(recv), 4):
            sensors_identifiers.append(int.from_bytes(recv[i:i+4], "little"))

        logger.info(f"Known sensors: {list(map(lambda x: f'0x{x:08X}', sensors_identifiers))}", self.__class__.__name__)

        return sensors_identifiers

    def read_sensor_info_raw(self, identifier: int) -> bytearray:
        packet = bytearray()
        packet.append(UsbCommand.GetSensorInfo.value)
        packet.append(0)
        packet += int_to_bytes(identifier)
        packet[1] = len(packet) - 2

        recv = MonitorStationDevice().write_read(packet)

        return recv[2:]

    def read_calibration_data(self, identifier: int) -> bytearray:
        packet = bytearray()
        packet.append(UsbCommand.GetSensorCalibData.value)
        packet.append(0)
        packet += int_to_bytes(identifier)
        packet[1] = len(packet) - 2

        recv = MonitorStationDevice().write_read(packet)

        return recv[2:]
    
    def read_sensor_last_data(self, identifier: int) -> bytearray:
        packet = bytearray()
        packet.append(UsbCommand.GetSensorLastData.value)
        packet.append(0)
        packet += int_to_bytes(identifier)
        packet[1] = len(packet) - 2

        recv = MonitorStationDevice().write_read(packet)

        return recv[2:]

    def read_available_measurement(self, wheather_station_id: int | None = None):
        pass

    # def read_sensor_data(self, sensor: Sensor):
    #     packet = bytearray(UsbCommand.GetSensorDataCount, 0)
    #     packet += int_to_bytes(sensor.identifier)
    #     packet[1] = len(packet)
    #     # recv = self.indoor_outdoor_receiver.write_read(packet)
        
    #     UsbCommand.GetSensorData

    def read_log(self) -> None:
        """Read logs from device and log to screen and to file"""
        """TODO"""
        pass
