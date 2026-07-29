from sqlmodel import Field, SQLModel
from typing import Annotated, Optional, ClassVar
from datetime import datetime, timezone

class CpuTempModel(SQLModel, table=True):
    __tablename__ = "cpu_temperature" # type: ignore[assignment]
    id: int | None = Field(default=None, primary_key=True)
    created_at: Optional[datetime] = Field(default_factory = lambda: datetime.now(timezone.utc))
    temp: float = Field(default=0.0)
