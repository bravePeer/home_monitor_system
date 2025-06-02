import mysql.connector
import dotenv
import os
from logger import logger
from time import sleep

dotenv.load_dotenv()
DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_NAME = os.getenv("DB_NAME")
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")

def connect_to_database():
    """ Try to connect to database with 5 times, 2 second delay """
    
    exception_to_raise = Exception
    for i in range(5):
        logger.info(f"Try to connect to dabase, try number: {i}", connect_to_database.__name__)
        try:
            db = mysql.connector.connect(host=DB_HOST, port=DB_PORT, user=DB_USER, password=DB_PASSWORD, database=DB_NAME)
            logger.info(f"Connected to database! Try number: {i}", connect_to_database.__name__)
            return db
        except Exception as e:
            logger.error("Can not connect to database!", connect_to_database.__name__)
            exception_to_raise = e
        sleep(2)
    raise exception_to_raise
        