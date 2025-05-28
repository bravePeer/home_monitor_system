from enum import Enum

class DaemonStatus(Enum):
    Stopped = 0
    Halted = 1
    Running = 2
    Error = 3