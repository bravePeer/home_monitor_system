from logger import logger
import time

"""

WheatherStationDevice <-> WhetherStationCommunicator
      Low level                 High level

"""

import usb.core
import usb.util
from logger import logger

def bytes_to_str(data: bytes | bytearray)->str:
    ret = ""
    for i in data:
        ret += f" {i:02X}"
    return ret

class WheatherStationDevice:
    def __init__(self, id_vendor: int, id_product: int):
        self.id_vendor = id_vendor
        self.id_product = id_product
        self.is_connected = False
        self.outendpoint = None
        self.inendpoint = None

    def configure_endpoints(self, dev) -> bool:
        cfg = dev.get_active_configuration() # Get configuration
        intf = cfg[(0, 0)]

        self.outendpoint = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match= \
                lambda e: \
                    usb.util.endpoint_direction(e.bEndpointAddress) == \
                    usb.util.ENDPOINT_OUT)

        self.inendpoint = usb.util.find_descriptor(
            intf,
            # match the first IN endpoint
            custom_match= \
                lambda e: \
                    usb.util.endpoint_direction(e.bEndpointAddress) == \
                    usb.util.ENDPOINT_IN)

        if self.outendpoint is None:
            logger.error("Can not configure OUT ENDPOINT!", self.__class__.__name__)
            return False

        if self.inendpoint is None:
            logger.error("Can not configure IN ENDPOINT!", self.__class__.__name__)
            return False

        logger.info("ENDPOINTs configured", self.__class__.__name__)
        return True

    def connect(self):
        logger.info("Connecting to device...", self.__class__.__name__)
        # dev = usb.core.find(idVendor=self.id_vendor, idProduct=self.id_product)
        dev = usb.core.find(idVendor=0x0000, idProduct=0x0001)

        if dev is None:
            logger.error("Device not found!", self.__class__.__name__)
            self.is_connected = False
            return

        if self.configure_endpoints(dev) is False:
            self.is_connected = False
            return

        self.is_connected = True
        logger.info("Connected!", self.__class__.__name__)
            
    def write_read(self, data: bytes, read_max_packet_size = 64) -> bytearray | None:
        if not self.is_connected:
            logger.error("Device not connected!", self.__class__.__name__)
            return None

        try:
            self.outendpoint.write(data)
            logger.info(f"Wrote: {bytes_to_str(data)}", self.__class__.__name__)
        except:
            logger.error("Can not write data!", self.__class__.__name__)
            raise

        ret = None
        try:
            ret = self.inendpoint.read(read_max_packet_size)
            logger.info(f"Read: {bytes_to_str(data)}", self.__class__.__name__)
        except:
            logger.error("Can not read data!", self.__class__.__name__)
            raise
            
        return ret

class WheatherStationCommunicator:
    def __init__(self, id_vendor: int, id_product: int):
        self.id_vendor = id_vendor
        self.id_product = id_product


    def check_device(self):
        pass

    def read_wheather_station_count(self):
        pass

    def read_available_mesurment(self, wheather_station_id: int | None = None):
        pass

    def read_log(self) -> None:
        """Read logs from device and log to screen and to file"""
        """TODO"""
        pass