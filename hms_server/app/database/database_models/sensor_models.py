from sqlmodel import Field, SQLModel, Relationship
from typing import Annotated, Optional, ClassVar
from datetime import datetime, timezone
from pydantic import ConfigDict
from sqlalchemy import Column, func
from sqlalchemy.dialects.postgresql import DOUBLE_PRECISION, BIGINT

# class SQLModel(SQLModel):
    # model_config = ConfigDict(arbitrary_types_allowed=True)

class SensorTypeModel(SQLModel, table=True):
    __tablename__ = "sensor_type" # type: ignore[assignment]
    id: int | None = Field(default=None, sa_column=Column(BIGINT(), primary_key=True))
    sensor_type_uid: int = Field(unique=True)
    name: str | None
    sensors: list["SensorModel"] = Relationship(back_populates="sensor_type")

class SensorModel(SQLModel, table=True):
    __tablename__ = "sensor" # type: ignore[assignment]
    id: int | None = Field(default=None, sa_column=Column(BIGINT(), primary_key=True))
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))
    sensor_uid: int | None = Field(default=None, unique=True)
    sensor_type_id: int | None = Field(default=None, foreign_key="sensor_type.id")
    sensor_type: SensorTypeModel | None = Relationship(back_populates="sensors")
    name: str
    desc: str


class SensorCalibrationModel(SQLModel, table=True):
    __tablename__ = "sensor_calibration" # type: ignore[assignment]
    id: int | None = Field(default=None, sa_column=Column(BIGINT(), primary_key=True))
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))
    sensor_id: int = Field(default=None, foreign_key="sensor.id")
    calibration_data: str # TODO Change to JSONB


class SensorMeasurementTypeModel(SQLModel, table=True):
    __tablename__ = "measurement_type" # type: ignore[assignment]
    id: int | None = Field(default=None, sa_column=Column(BIGINT(), primary_key=True))
    measurement_type_uid: int = Field(default=None, unique=True)
    name: str
    unit: str


class SensorMeasurementModel(SQLModel, table=True):
    __tablename__ = "measurement" # type: ignore[assignment]
    id: int | None = Field(default=None, sa_column=Column(BIGINT(), primary_key=True))
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))
    sensor_id: int | None = Field(default=None, foreign_key="sensor.id")
    measurement_type: int | None = Field(default=None, foreign_key="measurement_type.id")
    measurement: float | None = Field(default=None, sa_column=Column(DOUBLE_PRECISION()))