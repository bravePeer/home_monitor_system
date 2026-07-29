from sqlmodel import Session, SQLModel, create_engine
from os import getenv as os_getenv
from urllib.parse import quote_plus
from fastapi import Depends
from typing import Annotated
from .database_models import sensor_models

DB_LOGIN = os_getenv("DB_LOGIN", "postgres")
DB_PASSWORD = os_getenv("DB_PASSWORD", "password")
DB_HOST = os_getenv("DB_HOST", "127.0.0.1")
DB_PORT = os_getenv("DB_PORT", "3306")
DB_NAME = os_getenv("DB_NAME", "hms_db")

engine = create_engine(f"postgresql+psycopg://{quote_plus(DB_LOGIN)}:{quote_plus(DB_PASSWORD)}@{DB_HOST}:{DB_PORT}/{DB_NAME}", echo=True)

def drop_db_tables() -> bool:
    try:
        SQLModel.metadata.drop_all(engine)
    except Exception as e:
        print(f"Error {e}")
        return False
    return True

def create_db_and_tables() -> bool:
    try:
        SQLModel.metadata.create_all(engine)
    except Exception as e:
        print(f"Error {e}")
        return False
    return True

def get_session():
    with Session(engine) as session:
        yield session

SessionDep = Annotated[Session, Depends(get_session)]
