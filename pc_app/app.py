import time
from logger import logger
from wheather_station import WheatherStationDevice
import usb.core
import usb.util
import time
 
# find our device
dev = usb.core.find(idVendor=0x0000, idProduct=0x0001)

# was it found?
if dev is None:
    raise ValueError('Device not found')

logger.init()

logger.info("Wait")
device = WheatherStationDevice(id_vendor=0x0000, id_product=0x0001)
device.connect()
test_data = [0x10, 0x00]
device.write_read(test_data)