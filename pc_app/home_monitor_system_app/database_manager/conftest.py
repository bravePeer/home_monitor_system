import pytest
from database_manager.database_manager import DatabaseManagerMeta
import sys
sys.path.append("..")

@pytest.fixture(autouse=True)
def clean_singleton():
    DatabaseManagerMeta._instances = {}