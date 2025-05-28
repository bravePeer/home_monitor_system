class SensorExceptionGeneric(Exception):
    def __init__(self, message, errors):
        super().__init__(message)
        self.errors = errors

class SensorExceptionCalibrationData(SensorExceptionGeneric):
    def __init__(self, message = "Calibration data is not known or is invalid"):
        super().__init__(message, None)

class SensorExceptionDataEmpty(SensorExceptionGeneric):
    def __init__(self, message = "Data is empty"):
        super().__init__(message, None)