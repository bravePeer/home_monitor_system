class UsbExceptionGeneric(Exception):
    def __init__(self, message, errors):
        super().__init__(message)
        self.errors = errors


class UsbExceptionBadCommand(UsbExceptionGeneric):
    def __init__(self, message = "Bad command wrote to device"):
        super().__init__(message, None)


class UsbExceptionLenghtError(UsbExceptionGeneric):
    def __init__(self, message = "Wrong lenght"):
        super().__init__(message, None)


class UsbExceptionInternalError(UsbExceptionGeneric):
    def __init__(self, errors, message):
        tmessage = "Internal command error: " + message
        super().__init__(tmessage, errors)