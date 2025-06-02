import pytest
from sensor_manager.sensor_manager import SensorManager
from unittest.mock import Mock, MagicMock, patch
import sys
sys.path.append("..")
from sensor import sensor

class TestSensorManager():
    @patch('sensor_manager.sensor_manager.MonitorStationCommunicator')
    def test_get_not_known_sensors_identifiers(self, mock_monitor_station_communicator):

        mock_monitor_station_communicator.return_value.read_sensors_identifiers.return_value = [0x1, 0x2, 0x3, 0x4, 0x6]

        sm = SensorManager()
        sm.sensors.append(sensor.SensorTestClass(0x1))
        sm.sensors.append(sensor.SensorTestClass(0x4))
        sm.sensors.append(sensor.SensorTestClass(0x5))

        not_known_sensors_ids = SensorManager().get_not_known_sensors_identifiers()

        assert not_known_sensors_ids == [0x2, 0x3, 0x6]

    # @patch('sensor_manager.sensor_manager.MonitorStationCommunicator')
    # def test_get_new_sensors(self, mock_monitor_station_communicator):
        
    #     def side_effect(id: int):
    #         return {
    #             0x1 : bytearray()
    #         }
    #     mock_monitor_station_communicator.return_value.

