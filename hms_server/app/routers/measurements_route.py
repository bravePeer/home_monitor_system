from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Any, Optional
from datetime import datetime

from ..dependencies import get_token_header
from ..database.database import SessionDep

from ..database.database_models.sensor_models import SensorMeasurementTypeModel, SensorMeasurementModel
from ..database.sensor_query import query_sensor_read_by_uid
from ..database.measurements_query import query_measurement_type_read_all, query_measurement_type_create, query_measurement_type_read, query_measurement_create
# from ..sensor.sensor import sensor_get_measurement_type_by_sensor_type


class MeasurementGeneric(BaseModel):
    measurement_type_uid: int 
    measurement: Any

class MeasurementCreate(BaseModel):
    sensor_uid: int
    measurements: list[MeasurementGeneric]

class MeasuremetTypeCreate(BaseModel):
    measurement_type_uid: int
    name: str
    unit: str


class MeasurementCreateResponse(BaseModel):
    created_data: SensorMeasurementTypeModel | SensorMeasurementModel | None
    created: bool


router = APIRouter(
    prefix="/measurements",
    tags=["measurements"],
    # dependencies=[Depends(get_token_header)],
    responses={404: {"description": "Not found"}},
)


@router.get("/types")
def measurements_types_read(session: SessionDep):
    measurement_types = query_measurement_type_read_all(session)
    return measurement_types

@router.post("/types/create")
def measurements_type_create(measurement_type_req: MeasuremetTypeCreate, session: SessionDep):
    ret, created = query_measurement_type_create(
        measurement_type_req.measurement_type_uid,
        measurement_type_req.name,
        measurement_type_req.unit,
        session
    )
    return MeasurementCreateResponse(created_data=ret, created=created)



@router.post("/create")
def sensor_create_measurement(measurement: MeasurementCreate, session: SessionDep):
    sensor = query_sensor_read_by_uid(measurement.sensor_uid, session)

    if sensor is None:
        raise HTTPException(status_code=400, detail="Sensor with this id not exists")

    """ TODO Validate params, sensor with this type can generate measurements """
    # for key, value in sensor_data.data.items():
    #     if key not in SensorMeasurementTypeToCreateFunctionMap:
    #         raise HTTPException(status_code=400, detail=f"Invalid sensor data key: {key}")

    # if sensor_val.id is None:
    #     raise HTTPException(status_code=400, detail="Sensor with this id not exists")

    ret = []
    for meas in measurement.measurements:
        meas_type = query_measurement_type_read(meas.measurement_type_uid, session)
        if meas_type is None:
            raise HTTPException(status_code=400, detail="Measurement type with this id not exists")
        a = query_measurement_create(sensor, meas_type, meas.measurement, session)
        ret.append(a)

    return ret
