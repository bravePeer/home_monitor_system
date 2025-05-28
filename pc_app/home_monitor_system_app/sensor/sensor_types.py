from enum import Enum

class SensorType(Enum):
    Test = 0
    SimpleWheatherStation = 0x58
    Unknown = 0xff
    
    def __int__(self):
        return self.value