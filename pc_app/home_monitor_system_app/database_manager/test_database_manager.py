import pytest
from database_manager.database_manager import DatabaseManager
from unittest.mock import Mock, MagicMock
import sys
sys.path.append("..")
from sensor import sensor

class TestDatabaseManager:
    def test_table_name_syntax(self):
        db = Mock()
        cursor = MagicMock()
        db.cursor.return_value = cursor

        dm = DatabaseManager(db)
        result = dm._DatabaseManager__check_table_syntax("test_table_name")
        assert result == "test_table_name"

        with pytest.raises(Exception):
            dm._DatabaseManager__check_table_syntax(" test_table_name")

        with pytest.raises(Exception):
            dm._DatabaseManager__check_table_syntax("test_table_name -- ")

        with pytest.raises(Exception):
            dm._DatabaseManager__check_table_syntax("test_table_name--")

        with pytest.raises(Exception):
            dm._DatabaseManager__check_table_syntax(" test_table_name--*&%#$%awd")
    
    def test_table_no_exists(self):
        db = Mock()
        cursor = MagicMock()
        execute = MagicMock()
        db.cursor.return_value = cursor
        cursor.execute = execute

        # db.cursor.fetchone.return_value = [0]
        print(dir(db))
        dm = DatabaseManager(db)
        result = dm._DatabaseManager__check_table_exists("test_table_name")
        print(dir(db))
        assert result == False
        assert db.cursor.call_count == 1
        assert cursor.execute.call_count == 1

    def test_table_create(self):
        db = Mock()
        cursor = MagicMock()
        db.cursor.return_value = cursor

        db.cursor.fetchone.return_value = [0]
    
        sen = sensor.SensorTestClass()

        dm = DatabaseManager(db)
        dm._DatabaseManager__create_table("test_table", sen)
        # assert cursor.execute.call_count == 1
        # assert cursor.execute.call_args == "awdawd"
        

    # def test_table_exists(self):
    #     db = Mock()
    #     cursor = MagicMock()
    #     db.cursor.return_value = cursor

    #     db.cursor.fetchone.return_value = [0]

    #     dm = DatabaseManager(db)
    #     result = dm._DatabaseManager__check_table_exists("test_table_name")
    #     assert result == False