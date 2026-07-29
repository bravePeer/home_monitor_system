from functools import cache
import json

from os import getenv as os_getenv
HMS_SENSORS_CONFIG_FILE = os_getenv("HMS_SENSORS_CONFIG", "hms_sensors_config.json")

""" 
Info for future:
To clear cache: get_config.cache_clear() 
"""

@cache
def get_config():
    with open(HMS_SENSORS_CONFIG_FILE, encoding="utf-8") as f:
        return json.load(f)