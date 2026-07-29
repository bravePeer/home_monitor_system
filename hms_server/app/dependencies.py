from typing import Annotated
from fastapi import Header, HTTPException
from .database.sensor_query import query_sensor_read_by_uid

async def get_token_header(x_token: Annotated[str, Header()]):
    return
    if x_token != "fake-super-secret-token":
        raise HTTPException(status_code=400, detail="X-Token header invalid")


async def get_query_token(token: str):
    if token != "jessica":
        raise HTTPException(status_code=400, detail="No Jessica token provided")
