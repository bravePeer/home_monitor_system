from __future__ import annotations
import time
import sensor.sensor as sensor
from logger import logger
import mysql.connector
import re

class DatabaseManagerMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class DatabaseManager(metaclass=DatabaseManagerMeta):
    def __init__(self, db: mysql.connector.MySQLConnection):
        self.db = db

    def __check_table_syntax(self, table_name: str) -> str:
        table_name_tmp = re.fullmatch(r"\w+", table_name)
        if table_name_tmp is None:
            raise Exception("Regex error")
        table_name = table_name_tmp[0]
        return table_name

    def __check_table_exists(self, table_name: str):
        table_name = self.__check_table_syntax(table_name)
        
        cur = self.db.cursor()
        cur.execute(f"""SELECT COUNT(*)
            FROM information_schema.tables
            WHERE table_name = '{table_name}'"""
            )
        
        if cur.fetchone()[0] == 1:
            cur.close()
            return True
        
        cur.close()
        return False
    
    def __create_table(self, table_name: str, sensor: sensor.Sensor):
        table_name = self.__check_table_syntax(table_name)

        sfuncs = sensor.get_available_functions()
        if len(sfuncs) == 0:
            raise Exception("No sfuncs found!")
        
        columns = ""
        for sfunc in sfuncs:
            tmp = str(getattr(sensor, sfunc).__annotations__["return"])
            tmp = tmp[8:-2]
            columns += f", {sfunc} {tmp}"

        cur = self.db.cursor()
        command = f"CREATE TABLE {table_name} (update_time DATETIME {columns})"
        print(command)
        cur.execute(command)

        cur.close()

    def archive_data(self, sensor: sensor.Sensor):
        """Table name: sensor_sensorclass_id"""

        table_name = "sensor_" + sensor.__class__.__name__ + sensor.identifier.__str__()
        if self.__check_table_exists(table_name) is False:
            self.__create_table(table_name, sensor)

        sfuncs = sensor.get_available_functions()
        if len(sfuncs) == 0:
            raise Exception("No sfuncs found!")
        
        columns = ""
        for sfunc in sfuncs:
            columns += f"{sfunc},"  
        columns = columns[:-1]

        values = ""
        for sfunc in sfuncs:
            values += f"{str(getattr(sensor, sfunc)())},"
        values = values[:-1]

        cur = self.db.cursor()
        command = f"""INSERT INTO {table_name} ({columns}) VALUES ({values})"""
        print(command)
        cur.execute(command)

        cur.close()
        self.db.commit()


    def execute_command(self, command: str):
        logger.warning("This is only for testing!", self.execute_command.__name__)
        cur = self.db.cursor()
        cur.execute(command)
        ret = cur.fetchall()
        cur.close()
        return ret