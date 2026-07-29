from .database import SessionDep
from sqlmodel import select
from .database_models.sensor_models import SensorMeasurementModel, SensorMeasurementTypeModel, SensorModel
from sqlalchemy.exc import IntegrityError
from typing import Sequence

def query_measurement_type_read(measurement_type_uid: int, session: SessionDep):
    query = select(SensorMeasurementTypeModel).where(SensorMeasurementTypeModel.measurement_type_uid == measurement_type_uid)
    ret = session.exec(query).first()
    return ret

def query_measurement_type_create(measurement_type_uid: int, name: str, unit: str, session: SessionDep):
    """ If exists return existing, else create new and return it. 
        @return [model, created]
    """

    existing = query_measurement_type_read(measurement_type_uid, session)
    if existing:
        return existing, False

    new_measurement_type = SensorMeasurementTypeModel(
        measurement_type_uid=measurement_type_uid,
        name=name,
        unit=unit
    )

    try:
        session.add(new_measurement_type)
        session.commit()
        session.refresh(new_measurement_type)
        return new_measurement_type, True
    except IntegrityError:
        session.rollback()
        existing = query_measurement_type_read(measurement_type_uid, session)
        if existing:
            return existing, False
        raise

def query_measurement_create(sensor: SensorModel, meas_type: SensorMeasurementTypeModel , meas_value: float, session: SessionDep):
    meas = SensorMeasurementModel(
        sensor_id=sensor.id,
        measurement_type=meas_type.id,
        measurement=meas_value
    )

    try:
        session.add(meas)
        session.commit()
        session.refresh(meas)
        return meas, True
    except IntegrityError:
        session.rollback()
        raise

def query_measurement_type_read_all(session: SessionDep) -> Sequence[SensorMeasurementTypeModel]:
    query = select(SensorMeasurementTypeModel)
    ret = session.exec(query).all()
    return ret
