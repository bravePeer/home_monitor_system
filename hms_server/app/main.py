from dotenv import load_dotenv
load_dotenv()

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from .database.database import create_db_and_tables, SessionDep, drop_db_tables
from .database.database_models.rpi_models import CpuTempModel

from .routers import sensors_route
from .routers import measurements_route
from .routers import init_route

app = FastAPI()

app.include_router(sensors_route.router)
app.include_router(measurements_route.router)
app.include_router(init_route.router)

@app.get("/create-db")
def create_db():
    print("Dropping tables...", end="")
    drop_db_tables()
    print("DONE")
    print("Creating tables...", end="")
    was_created = create_db_and_tables()
    print("DONE")
    return {"Created": was_created}


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

