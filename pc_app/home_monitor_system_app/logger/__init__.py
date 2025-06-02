from pathlib import Path
import colorama as col
from colorama import Fore, Style
from datetime import datetime
import inspect
import dotenv
import os

dotenv.load_dotenv()

LOG_FILE_MAX_SIZE = int(os.getenv("LOG_FILE_MAX_SIZE"))
LOG_PATH0 = os.getenv("LOG_PATH0")
LOG_PATH1 = os.getenv("LOG_PATH1")

class LoggerMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class Logger(metaclass=LoggerMeta):
    """ Responsible for store and print on terminal logs """
    def __init__(self):
        self.initialized = False
        self.silent = False
        self.lines = 0

    def init(self, path0: str, path1: str):
        if self.initialized is True:
            return
        self.path0 = Path(path0)
        self.path1 = Path(path1)
        self.path0.parent.mkdir(parents=True, exist_ok=True)
        if not self.path0.exists():
            t = open(self.path0, "tw")
            t.close()
        if not self.path1.exists():
            t = open(self.path1, "tw")
            t.close()

        #Check newer log
        last_log0 = self.get_time_of_last_log(self.path0)
        last_log1 = self.get_time_of_last_log(self.path1)
        if last_log0 >= last_log1:
            self.log_file = open(self.path0, "ta+")
        else:
            self.log_file = open(self.path1, "ta+")

        col.init()
        self.initialized = True

    def get_time_of_last_log(self, path: Path) -> datetime:
        file = open(path, "tr")
        line = file.readline()
        for l in file.readlines():
            if l == "":
                break
            line = l
        words = line.split(" ")
        try:
            date_str = words[1] + " " + words[2][:-1]
        except:
            return datetime(2025, 1, 1, 1, 1, 1, 1)
        return datetime.strptime(date_str, '%Y.%m.%d %H:%M:%S')

    def change_file(self):
        if self.log_file.name == self.path0.name:
            self.log_file.close()
            self.log_file = open(self.path1, "tw+")
        else:
            self.log_file.close()
            self.log_file = open(self.path0, "tw+")
        self.lines = 0

    def __log(self, msg:str):
        msg = msg.replace("time", datetime.now().strftime('%Y.%m.%d %H:%M:%S'))
        msg += "\n"
        self.lines = self.lines + 1
        self.log_file.write(msg)
        self.log_file.flush()
        
        if self.silent is False:
            print(msg, end='')

        if self.lines > LOG_FILE_MAX_SIZE:
            self.change_file()
        
    def warning(self, msg: str, module: str = None):
        if module is None:
            m_msg = f"{Fore.YELLOW}[ time ] [ WARNING ] " + msg + f"{Style.RESET_ALL}"
        else:
            m_msg = f"{Fore.YELLOW}[ time ] [ WARNING ] [ {module} ] " + msg + f"{Style.RESET_ALL}"
        self.__log(m_msg)

    def error(self, msg: str, module: str = None):
        if module is None:
            m_msg = f"{Fore.RED}[ time ] [ ERROR   ] [ {inspect.stack()[1].function} ] [ {inspect.stack()[1].filename} : {inspect.stack()[1].lineno} ] " + msg + f"{Style.RESET_ALL}"
        else:
            m_msg = f"{Fore.RED}[ time ] [ ERROR   ] [ {module} ] [ {inspect.stack()[1].filename} : {inspect.stack()[1].lineno} ] " + msg + f"{Style.RESET_ALL}"
        self.__log(m_msg)

    def info(self, msg: str, module: str = None):
        if module is None:
            m_msg = f"{Fore.LIGHTWHITE_EX}[ time ] [ INFO    ] " + msg + f"{Style.RESET_ALL}"
        else:
            m_msg = f"{Fore.LIGHTWHITE_EX}[ time ] [ INFO    ] [ {module} ] " + msg + f"{Style.RESET_ALL}"
        self.__log(m_msg)

    def deinit(self):
        self.initialized = False
        self.log_file.flush()
        self.log_file.close()
logger = Logger()