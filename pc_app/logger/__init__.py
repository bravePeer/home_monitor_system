from pathlib import Path
import colorama as col
from colorama import Fore, Style
from datetime import datetime

class Logger:
    """ Responsible for store and print on terminal logs """
    def __init__(self):
        self.initialized = False

    def init(self, path: str = "log.log"):
        if self.initialized is True:
            return
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = open(self.path, "ta+")
        col.init()
        self.initialized = True

    def __log(self, msg:str):
        msg = msg.replace("time", datetime.now().strftime('%Y.%m.%d %H:%M:%S'))
        msg += "\n"
        self.log_file.write(msg)
        print(msg, end='')

    def warning(self, msg: str, module: str = None):
        if module is None:
            m_msg = f"{Fore.YELLOW}[ time ] [ WARNING ] " + msg + f"{Style.RESET_ALL}"
        else:
            m_msg = f"{Fore.YELLOW}[ time ] [ WARNING ] [ {module} ] " + msg + f"{Style.RESET_ALL}"
        self.__log(m_msg)

    def error(self, msg: str, module: str = None):
        if module is None:
            m_msg = f"{Fore.RED}[ time ] [ ERROR   ] " + msg + f"{Style.RESET_ALL}"
        else:
            m_msg = f"{Fore.RED}[ time ] [ ERROR   ] [ {module} ] " + msg + f"{Style.RESET_ALL}"
        self.__log(m_msg)

    def info(self, msg: str, module: str = None):
        if module is None:
            m_msg = f"{Fore.LIGHTWHITE_EX}[ time ] [ INFO    ] " + msg + f"{Style.RESET_ALL}"
        else:
            m_msg = f"{Fore.LIGHTWHITE_EX}[ time ] [ INFO    ] [ {module} ] " + msg + f"{Style.RESET_ALL}"
        self.__log(m_msg)

logger = Logger()