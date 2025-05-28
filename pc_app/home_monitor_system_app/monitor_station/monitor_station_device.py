import usb.core
import usb.util
from logger import logger
from .usb_commands import UsbErrorCode, UsbCommandErrorCode, usbErrorMask
from .usb_error import *
from .utils import bytes_to_str
import libusb_package
import usb.core
import usb.backend.libusb1

class MonitorStationDeviceMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class MonitorStationDevice(metaclass=MonitorStationDeviceMeta):
    def __init__(self):
        self.id_vendor = 0x0000
        self.id_product = 0x0001
        self.is_connected = False
        self.outendpoint = None
        self.inendpoint = None
        self.device = None

    # def __del__(self):
    #     try:
    #         self.disconnect()
    #     except Exception as e:
    #         pass

    def set_vid_pid(self, id_vendor: int, id_product: int):
        self.id_vendor = id_vendor
        self.id_product = id_product

    def configure_endpoints(self, device) -> bool:
        cfg = device.get_active_configuration() # Get configuration
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
        if self.is_connected:
            logger.info("Already connected!", self.__class__.__name__)
            return

        logger.info("Connecting to device...", self.__class__.__name__)
        # dev = usb.core.find(idVendor=self.id_vendor, idProduct=self.id_product)
        libusb1_backend = libusb1_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
        self.device = usb.core.find(idVendor=0x0000, idProduct=0x0001, backend=libusb1_backend)

        if self.device is None:
            logger.error("Device not found!", self.__class__.__name__)
            self.is_connected = False
            return

        if self.configure_endpoints(self.device) is False:
            self.is_connected = False
            return

        self.is_connected = True
        logger.info("Connected!", self.__class__.__name__)

    def disconnect(self):
        if not self.is_connected:
            logger.info("Already disconnected!", self.__class__.__name__)
            return
        
        logger.info("Disconnecting from device...", self.__class__.__name__)
        usb.util.dispose_resources(self.device)
        self.device.reset()
        self.is_connected = False   
    
    def _check_received_data(self, data: bytes):
        if len(data) != data[1]:
            raise UsbExceptionLenghtError(None, "Host got wrong lenght")

        error_flags = data[0] & usbErrorMask
        if error_flags == 0:
            return

        if error_flags == UsbErrorCode.BadCommandError:
            raise UsbExceptionBadCommand
        
        if error_flags == UsbErrorCode.LenghtError:
            raise UsbExceptionLenghtError(None, "Device got wrong lenght")
         
        if len(data) != 6:
            raise UsbExceptionLenghtError
        
        internal_error_code = int.from_bytes(data[2:6], "little")
        
        raise UsbExceptionInternalError(internal_error_code, UsbCommandErrorCode(internal_error_code).name)

    # Return whole packet
    def write_read(self, data: bytes, read_max_packet_size = 64) -> bytearray | None:
        if not self.is_connected:
            logger.warning("Device not connected!", self.__class__.__name__)
            logger.info("Try to automatically connect to device...", self.__class__.__name__)
            self.connect()
            if not self.is_connected:
                raise UsbExceptionGeneric("Device not connected!", None)

        try:
            self.outendpoint.write(data)
            logger.info(f"Wrote: {bytes_to_str(data)}", self.__class__.__name__)
        except:
            logger.error("Can not write data!", self.__class__.__name__)
            raise

        ret = None
        try:
            ret = self.inendpoint.read(read_max_packet_size)
            logger.info(f"Read: {bytes_to_str(ret)}", self.__class__.__name__)
        except:
            logger.error("Can not read data!", self.__class__.__name__)
            raise
            
        try:
            self._check_received_data(ret)
        except Exception as e:
            logger.error(f"Error: '{e}' during checking received packet!", self.__class__.__name__)
            raise e
        
        return ret
