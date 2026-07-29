from .database import SessionDep
from sqlmodel import select
from .database_models.sensor_models import SensorModel, SensorTypeModel
from sqlalchemy.exc import IntegrityError
from typing import Sequence

def query_sensor_type_read_all(session: SessionDep) -> Sequence[SensorTypeModel] | None:
    query = select(SensorTypeModel)
    ret = session.exec(query).all()
    return ret

def query_sensor_type_read_by_uid(sensor_type_uid: int, session: SessionDep) -> SensorTypeModel | None:
    query = select(SensorTypeModel).where(SensorTypeModel.sensor_type_uid == sensor_type_uid)
    ret = session.exec(query).first()
    return ret

def query_sensor_type_create(sensor_type_uid: int, sensor_type_name: str, session: SessionDep) ->  tuple[SensorTypeModel | None, bool]:
    """ If exists return existing, else create new and return it. 
        @return [model, created]
    """

    existing = query_sensor_type_read_by_uid(sensor_type_uid, session)
    if existing:
        return existing, False

    new_sensor_type = SensorTypeModel(
        sensor_type_uid=sensor_type_uid,
        name=sensor_type_name
    )

    try:
        session.add(new_sensor_type)
        session.commit()
        session.refresh(new_sensor_type)
        return new_sensor_type, True
    except IntegrityError:
        session.rollback()
        existing = query_sensor_type_read_by_uid(sensor_type_uid, session)
        if existing:
            return existing, False
        raise

def query_sensor_read_all(session: SessionDep) -> Sequence[SensorModel]:
    query = select(SensorModel).join(SensorModel.sensor_type)
    ret = session.exec(query).all()
    return ret

def query_sensor_read_by_uid(sensor_uid: int, session: SessionDep) -> SensorModel | None:
    query = select(SensorModel).where(SensorModel.sensor_uid == sensor_uid).join(SensorModel.sensor_type)
    ret = session.exec(query).first()
    return ret

def query_sensor_create(sensor_uid: int, sensor_type_uid: int, name: str, desc: str, session: SessionDep) -> tuple[SensorModel | None, bool]:
    """ If exists return existing, else create new and return it. 
        @return [model, created]
    """

    existing = session.exec(
        select(SensorModel)
        .where(SensorModel.sensor_uid == sensor_uid)
    ).first()
    if existing:
        return existing, False

    sensor_type = query_sensor_type_read_by_uid(sensor_type_uid, session)
    if sensor_type is None or sensor_type.id is None:
        raise ValueError("Invalid sensor type uid")

    sensor = SensorModel(
        sensor_uid=sensor_uid,
        sensor_type_id=sensor_type.id,
        name=name,
        desc=desc
    )

    try:
        session.add(sensor)
        session.commit()
        session.refresh(sensor)
        return sensor, True

    except IntegrityError:
        session.rollback()
        raise