"""Daemon is responsible for automate reading and storing data"""
"d"
import os
import time
import sys
from psutil import pid_exists
from socket import socket, AF_INET, SOCK_STREAM
import select
import dotenv

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from logger import logger
from daemon_status import DaemonStatus
from monitor_station import monitor_station_communicator

from database_manager import database_manager, database_connector

dotenv.load_dotenv()

LOG_DAEMON_PATH0 = os.getenv("LOG_DAEMON_PATH0")
LOG_DAEMON_PATH1 = os.getenv("LOG_DAEMON_PATH1")

DAEMON_PATH_PID_FILE = os.getenv("DAEMON_PATH_PID_FILE")
DAEMON_HOST = os.getenv("DAEMON_HOST")
DAEMON_PORT = int(os.getenv("DAEMON_PORT"))

class Daemon:
    class DaemonConfig:
        reading_interval = int(os.getenv("DEAMON_DEFAULT_READING_INTERVAL")) 

    def __init__(self):
        self.time_now = 0
        self.config = self.DaemonConfig
        self.status = DaemonStatus.Stopped
        self.soc = socket(AF_INET, SOCK_STREAM)
        try:
            self.soc.bind((DAEMON_HOST, DAEMON_PORT))
            self.soc.listen(1)
            self.soc.setblocking(False)
        except Exception as e:
            logger.error("Cannot bind address! Stopping...")
            return
        logger.info(f"Daemon started! PID: {str(os.getpid())}", self.__class__.__name__)
        self.status = DaemonStatus.Running

        try:
            time.sleep(60)
            self.db = database_connector.connect_to_database()
            self.dm = database_manager.DatabaseManager(self.db)
        except:
            logger.error("Error during connecting to database!")
            self.status = DaemonStatus.Error

    def process_command(self, command: str):
        if command == b"stop":
            self.status = DaemonStatus.Stopped
        elif command == b"halt":
            self.status = DaemonStatus.Halted
        elif command == b"run":
            self.status = DaemonStatus.Running

    def work(self):
        readable, writable, exceptional = select.select([self.soc], [], [self.soc], 1)
        
        for s in readable:
            if s is self.soc:
                conn, addr = self.soc.accept()
                with conn:
                    logger.info(f"Connected by: {str(addr)}")
                    while True:
                        recv = conn.recv(1024)
                        if not recv: 
                            break
                        self.process_command(recv)
                    logger.info(f"Dissconnected by: {str(addr)}")
        
        if self.status is not DaemonStatus.Running:
            return
        
        if time.time() - self.time_now < self.config.reading_interval:
            return
            
        monitor_station_communicator.MonitorStationCommunicator().read_sensors_count()
        # Read devices measurements
        # Write to db
        
        self.time_now = time.time()
            

def start_daemon():
    if is_deamon_alive():
        logger.error("Daemon is running! Exiting...", start_daemon.__name__)
        exit(-1)

    with open(DAEMON_PATH_PID_FILE, "w") as f:
        f.write(str(os.getpid()))

    daemon = Daemon()

    while daemon.status is not DaemonStatus.Stopped:
        daemon.work()
    
    logger.info(f"Daemon stopped! PID: {str(os.getpid())}", start_daemon.__name__)                    

def stop_daemon():
    try:
        with open(DAEMON_PATH_PID_FILE, "r") as f:
            pid = int(f.read())
            
        s = socket.socket()
        s.connect((DAEMON_HOST, DAEMON_PORT))
        s.sendall(b"stop")
        s.close()

        logger.info(f"Daemon stopping... PID: {pid}", stop_daemon.__name__)
        os.remove(DAEMON_PATH_PID_FILE)
    except FileNotFoundError:
        print("Nie znaleziono pliku PID.")
    except ProcessLookupError:
        print("Proces już nie działa.")

def is_deamon_alive() -> bool:
    try:
        with open(DAEMON_PATH_PID_FILE, "r") as f:
            pid = int(f.readline())
        if not pid_exists(pid):
            return False
        return True
    except FileNotFoundError:
        pass
    return False

if __name__ == "__main__":
    logger.silent = True
    logger.init(LOG_DAEMON_PATH0, LOG_DAEMON_PATH1)
    try:
        start_daemon()
    except Exception as e:
        logger.error(e)
