import mysql.connector
import dotenv
import os
from logger import logger

dotenv.load_dotenv()
DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_NAME = os.getenv("DB_NAME")
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")

def connect_to_database():
    try:
        db = mysql.connector.connect(host=DB_HOST, port=DB_PORT, user=DB_USER, password=DB_PASSWORD, database=DB_NAME)
        logger.info("Connected to database!", connect_to_database.__name__)
        return db
    except Exception as e:
        logger.error("Can not connect to database!", connect_to_database.__name__)
        raise e