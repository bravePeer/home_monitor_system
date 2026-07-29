from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from ..database.database import create_db_and_tables, SessionDep, drop_db_tables
from ..database.measurements_query import query_measurement_type_create
from ..database.sensor_query import query_sensor_type_create

from os import getenv as os_getenv
HMS_SENSORS_CONFIG_FILE = os_getenv("HMS_SENSORS_CONFIG", "hms_sensors_config.json")

class InitAppSensorType(BaseModel):
    uid: int
    name: str

class InitAppSensor(BaseModel):
    name: str
    desc: str
    type: InitAppSensorType
    measurements: list[str | int]
    processor: str | None

class InitAppMeasurement(BaseModel):
    uid: int
    name: str
    short_name: str
    unit: str

class InitAppRequest(BaseModel):
    sensors: list[InitAppSensor]
    measurements: list[InitAppMeasurement]

router = APIRouter(
    prefix="/init",
    tags=["init"],
    # dependencies=[Depends(get_token_header)],
    responses={404: {"description": "Not found"}},
)


@router.post("/initapp")
def init_app(sensors_config: InitAppRequest, session: SessionDep):
    print("Dropping tables...", end="")
    drop_db_tables()
    print("DONE")
    print("Creating tables...", end="")
    was_created = create_db_and_tables()
    print("DONE")

    if not was_created:
        raise HTTPException(status_code=400, detail="Error initializing databases!")

    with open(HMS_SENSORS_CONFIG_FILE, "w+") as file:
        file.write(sensors_config.model_dump_json())

    for meas in sensors_config.measurements:
        _, created = query_measurement_type_create(
            meas.uid,
            meas.name,
            meas.unit,
            session
        )
        if not created:
            raise HTTPException(status_code=400, detail="Error during creating!")

    for sensor_type in sensors_config.sensors:
        _, created = query_sensor_type_create(
            sensor_type.type.uid,
            sensor_type.type.name,
            session
        )
        if not created:
            raise HTTPException(status_code=400, detail="Error during creating!")

