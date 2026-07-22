from sqlmodel import Session, SQLModel, create_engine
from os import getenv as os_getenv
from fastapi import Depends
from typing import Annotated

from .database_models.rpi_models import CpuTempModel
from .database_models.sensor_models import SensorTypeModel, SensorIdentifierModel, SensorCalibrationModel, SensorDataTemperatureModel, SensorDataHumidityModel, SensorDataPressureModel

engine = create_engine(f"mysql+pymysql://{os_getenv('DB_LOGIN')}:{os_getenv('DB_PASSWORD')}@{os_getenv('DB_HOST')}:{os_getenv('DB_PORT')}/{os_getenv('DB_NAME')}")

def drop_db_tables() -> bool:
    try:
        SQLModel.metadata.drop_all(engine)
    except Exception as e:
        return False
    return True

def create_db_and_tables():
    SQLModel.metadata.create_all(engine)

def get_session():
    with Session(engine) as session:
        yield session

SessionDep = Annotated[Session, Depends(get_session)]
