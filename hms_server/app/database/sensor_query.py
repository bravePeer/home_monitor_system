from .database import SessionDep
from sqlmodel import select
from .database_models.sensor_models import SensorIdentifierModel, SensorTypeModel


def validate_sensor_id(sensor_id: int) -> bool:
    # TODO Validate senssor_id
    # raise ValueError("Invalid sensor id")
    return True


def get_sensor_type(sensor_type_id: int, session: SessionDep) -> SensorTypeModel | None:
    query = select(SensorTypeModel).where(SensorTypeModel.sensor_type_id == sensor_type_id)
    print(query)
    ret = session.exec(query).first()
    return ret

def create_sensor_type(sensor_type_id: int, sensor_type_name: str, session: SessionDep) -> SensorTypeModel:
    """ If exists return existing, else create new and return it. """
    old_sensor_type = get_sensor_type(sensor_type_id, session)
    if old_sensor_type is not None:
        return old_sensor_type

    new_sensor_type = SensorTypeModel(sensor_type_id=sensor_type_id, name=sensor_type_name)
    session.add(new_sensor_type)
    session.commit()
    session.refresh(new_sensor_type)
    return new_sensor_type

def get_all_sensors(session: SessionDep):
    query = select(SensorIdentifierModel, SensorTypeModel).join(SensorTypeModel, SensorIdentifierModel.sensor_type_id == SensorTypeModel.id)
    ret = session.exec(query).all()
    return ret

def get_sensor(sensor_id: int, session: SessionDep) -> SensorIdentifierModel | None:
    validate_sensor_id(sensor_id)

    query = select(SensorIdentifierModel).where(SensorIdentifierModel.sensor_id == sensor_id)
    ret = session.exec(query).first()

    return ret

def query_create_sensor(sensor_id: int, sensor_type_id: int, session: SessionDep) -> SensorIdentifierModel | None:
    validate_sensor_id(sensor_id)

    sensor_type = get_sensor_type(sensor_type_id, session)
    if sensor_type is None:
        raise ValueError("Invalid sensor type id")

    assert sensor_type.id is not None

    new_sensor = SensorIdentifierModel(sensor_id=sensor_id, sensor_type_id=sensor_type.id)
    session.add(new_sensor)
    session.commit()
    session.refresh(new_sensor)

    return new_sensor