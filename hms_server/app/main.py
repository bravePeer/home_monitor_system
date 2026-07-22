from dotenv import load_dotenv
load_dotenv()

from fastapi import FastAPI, HTTPException, Query

from typing import Annotated, Optional
from datetime import datetime, timezone

from pydantic import BaseModel
from typing import Tuple, Any

from .sensor.sensor_type import SensorType

from .database.database import create_db_and_tables, SessionDep, drop_db_tables
from .database.database_models.rpi_models import CpuTempModel

app = FastAPI()


@app.get("/create-db")
def create_db():
    drop_db_tables()
    create_db_and_tables()
    return {"Created": "DB"}


@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/status")
def get_server_status():
    return {"status": "ok"}


@app.post("/cpu_temp")
def create_cpu_temp(cpu_temp: CpuTempModel, session: SessionDep) -> CpuTempModel:
    session.add(cpu_temp)
    session.commit()
    session.refresh(cpu_temp)
    return cpu_temp


class SensorCreate(BaseModel):
    sensor_id: int
    sensor_type: SensorType
    calibration_data: Optional[str] = None

class SensorData(BaseModel):
    sensor_id: int
    sensor_type: SensorType
    data: dict[str, Any]
    # temperature_celsius: Optional[float] = None
    # humidity_percent: Optional[float] = None
    # pressure_pa: Optional[float] = None
    # battery_voltage: Optional[float] = None

class SensorDataResponse(BaseModel):
    msg: str

class SensorsAllResponsePart(BaseModel):
    id: int
    sensor_id: int
    sensor_type_id: int
    sensor_type_name: str
    created_at: datetime

class SensorsAllResponse(SensorDataResponse):
    sensors: list[SensorsAllResponsePart]


from .database.database_models.sensor_models import SensorIdentifierModel, SensorTypeModel
from .database.sensor_query import get_sensor, create_sensor_type, query_create_sensor, get_all_sensors
from .sensor.sensor import sensor_get_measurement_type_by_sensor_type
from .sensor.sensor_measurement_type import SensorMeasurementType, SensorMeasurementTypeToCreateFunctionMap

# @app.post("/sensor/type")
# def create_sensor_type( session: SessionDep) -> SensorTypeModel:
#     return create_sensor_type(sensor_type_id, sensor_type_name, session)

@app.get("/sensors")
def sensors_get(session: SessionDep) -> SensorsAllResponse:
    sensors = get_all_sensors(session)

    sensors_list: list[SensorsAllResponsePart] = []

    for sensor, sensor_type in sensors:
        if sensor.created_at is None or sensor.id is None:
            continue

        sensors_list.append(SensorsAllResponsePart(
            id=sensor.id,
            sensor_id=sensor.sensor_id,
            sensor_type_id=sensor_type.sensor_type_id,
            sensor_type_name=sensor_type.name,
            created_at=sensor.created_at 
        ))

    return SensorsAllResponse(msg="Sensors retrieved successfully", sensors=sensors_list)

@app.post("/sensor/create")
def sensor_create(sensor: SensorCreate, session: SessionDep):
    sensor_val = get_sensor(sensor.sensor_id, session)
    if sensor_val is not None:
        print(sensor_val)
        raise HTTPException(status_code=400, detail="Sensor with this id already exists")

    sensor_type = create_sensor_type(sensor.sensor_type.value, "", session)
    if sensor_type.id is None or sensor_type.sensor_type_id is None:
        raise HTTPException(status_code=500, detail="Failed to create sensor type")

    new_sensor = query_create_sensor(sensor.sensor_id, sensor_type.sensor_type_id, session)
    return new_sensor
    # return SensorDataResponse(msg="Sensor data received and processed")


@app.post("/sensor/data/create")
def sensor_data_create(sensor_data: SensorData, session: SessionDep) -> SensorDataResponse:
    sensor_val = get_sensor(sensor_data.sensor_id, session)
    if sensor_val is None:
        print(sensor_val)
        raise HTTPException(status_code=400, detail="Sensor with this id not exists")

    """ Validate params """
    for key, value in sensor_data.data.items():
        if key not in SensorMeasurementTypeToCreateFunctionMap:
            raise HTTPException(status_code=400, detail=f"Invalid sensor data key: {key}")

    if sensor_val.id is None:
        raise HTTPException(status_code=400, detail="Sensor with this id not exists")

    for key, value in sensor_data.data.items():
        print(f"{key}: {value}")
        SensorMeasurementTypeToCreateFunctionMap[SensorMeasurementType(key)](sensor_val.id, value, session)

    return SensorDataResponse(msg="Sensor data received and processed")
