from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Any, Optional
from datetime import datetime

from ..dependencies import get_token_header
from ..database.database import SessionDep
# from ..sensor.sensor_type import SensorType

from ..database.database_models.sensor_models import SensorTypeModel, SensorModel
from ..database.sensor_query import query_sensor_read_all, query_sensor_type_create, query_sensor_create,  query_sensor_type_read_all



class SensorTypeCreate(BaseModel):
    sensor_type_uid: int
    name: str | None

class SensorCreate(BaseModel):
    sensor_uid: int
    sensor_type_uid: int
    name: str | None
    desc: str | None
    calibration_data: Optional[str] = None

class SensorCreateResponse(BaseModel):
    created_data: SensorTypeModel | SensorModel | None
    created: bool


router = APIRouter(
    prefix="/sensors",
    tags=["sensors"],
    # dependencies=[Depends(get_token_header)],
    responses={404: {"description": "Not found"}},
)


@router.get("/")
async def read_sensors(session: SessionDep) -> list[tuple[SensorModel, SensorTypeModel]]:
    sensors = query_sensor_read_all(session)

    ret = []
    for sensor in sensors:
        ret.append((sensor, sensor.sensor_type))

    return ret

@router.get("/types")
async def read_sensors_type(session: SessionDep):
    sensors = query_sensor_type_read_all(session)
    return sensors

@router.post("/types/create")
async def create_sensors_type(sensor_type: SensorTypeCreate, session: SessionDep):
    name = "" # TODO Load default name for this sensor type uid
    if sensor_type.name is not None:
        name = sensor_type.name

    ret, created = query_sensor_type_create(sensor_type.sensor_type_uid, name, session)
    
    return SensorCreateResponse(created_data=ret, created=created)


@router.post("/create")
def sensor_create(sensor: SensorCreate, session: SessionDep):

    # TODO Load default name and desc
    name = ""
    desc = ""
    if sensor.name:
        name = sensor.name
    if sensor.desc:
            desc = sensor.desc

    ret, created = query_sensor_create(
        sensor.sensor_uid, 
        sensor.sensor_type_uid, 
        name,
        desc,
        session
        )

    return SensorCreateResponse(created_data=ret, created=created)

