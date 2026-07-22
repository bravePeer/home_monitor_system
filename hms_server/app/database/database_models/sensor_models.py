from sqlmodel import Field, SQLModel
from typing import Annotated, Optional, ClassVar
from datetime import datetime, timezone
from sqlalchemy import Column
from sqlalchemy.dialects.mysql import SMALLINT, MEDIUMINT, INTEGER

class SensorTypeModel(SQLModel, table=True):
    __tablename__ = "sensor_type" # type: ignore[assignment]
    id: int | None = Field(default=None, primary_key=True)
    sensor_type_id: int = Field(unique=True)
    name: str


class SensorIdentifierModel(SQLModel, table=True):
    __tablename__ = "sensor_identifier" # type: ignore[assignment]
    id: int | None = Field(default=None, primary_key=True)
    sensor_id: int = Field(unique=True)
    sensor_type_id: int = Field(default=None, foreign_key="sensor_type.id")
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))


class SensorCalibrationModel(SQLModel, table=True):
    __tablename__ = "sensor_calibration" # type: ignore[assignment]
    id: int | None = Field(default=None, primary_key=True)
    sensor_id: int = Field(default=None, foreign_key="sensor_identifier.id")
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))
    modified_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))
    calibration_data: str


class SensorDataGenericModel(SQLModel):
    """ Generic model for sensor data, not tied to a specific sensor type. """
    id: int | None = Field(default=None, primary_key=True)
    sensor_id: int = Field(default=None, foreign_key="sensor_identifier.id")
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))


class SensorDataTemperatureModel(SensorDataGenericModel, table=True):
    __tablename__ = "sensor_data_temperature" # type: ignore[assignment]
    temperature_raw: int = Field(sa_column=Column("temperature_celsius", MEDIUMINT), description="Temperature in Celsius multiplied by 1000 (e.g., 25.123°C is stored as 25123)")

    @property
    def temperature_celsius(self) -> float:
        return self.temperature_raw / 1000

    @temperature_celsius.setter
    def temperature_celsius(self, value: float) -> None:
        self.temperature_raw = int(round(value * 1000))

class SensorDataHumidityModel(SensorDataGenericModel, table=True):
    __tablename__ = "sensor_data_humidity" # type: ignore[assignment]
    humidity_raw: int = Field(sa_column=Column("humidity_percent", SMALLINT), description="Relative humidity percentage multiplied by 100 (e.g., 45.67%% is stored as 4567)")

    @property
    def humidity_percent(self) -> float:
        return self.humidity_raw / 1000

    @humidity_percent.setter
    def humidity_percent(self, value: float) -> None:
        self.humidity_raw = int(round(value * 100))


class SensorDataPressureModel(SensorDataGenericModel, table=True):
    __tablename__ = "sensor_data_pressure" # type: ignore[assignment]
    pressure_raw: int = Field("pressure_pa", sa_column=Column(INTEGER), description="Pressure in Pa multiplied by 100")

    @property
    def pressure_pa(self) -> float:
        return self.pressure_raw / 100

    @pressure_pa.setter
    def pressure_pa(self, value: float) -> None:
        self.humidity_raw = int(round(value * 100))


class SensorDataBatteryVoltageModel(SensorDataGenericModel, table=True):
    __tablename__ = "sensor_data_battery_voltage" # type: ignore[assignment]
    voltage_raw: int = Field("voltage", sa_column=Column(MEDIUMINT), description="Battery voltage in volts multiplied by 1000 (e.g., 3.7V is stored as 3700)")

    @property
    def voltage(self) -> float:
        return self.voltage_raw / 1000

    @voltage.setter
    def voltage(self, value: float) -> None:
        self.voltage_raw = int(round(value * 1000))
