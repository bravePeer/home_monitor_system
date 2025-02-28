#if defined(RP2040)
#include <pico/stdlib.h>

/// @brief This function is used to describe args
/// @param rxBuf Raw rx buffer, includes command and length bytes
/// @param rxLen Length of rxBuf
/// @param txBuf Data to write to host not include command and length bytes
/// @param txLen Length of data to write to host not include command and lenght bytes
/// @return Return command byte to write to host
uint8_t processUsbGeneric(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{

    return rxBuf[0];
}

uint8_t processUsbCommandAvailableData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    uint16_t* availableData = reinterpret_cast<uint16_t*>(&txBuf[0]);
    *availableData = usbData.dataLen;
    txLen = 2;
    return rxBuf[0];
}

uint8_t processUsbCommandGetData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    memcpy(txBuf, usbData.data, usbData.dataLen);
    txLen = usbData.dataLen;
    return rxBuf[0];
}

uint8_t processUsbCommandGetDataFromSensor(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen > )

    return rxBuf[0];
}

uint8_t processUsbCommands(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    UsbCommand usbCommand = static_cast<UsbCommand>(rxBuf[0]);
    switch (usbCommand)
    {
    case UsbCommand::AvailableData:
        return processUsbCommandAvailableData(rxBuf, rxLen, txBuf, txLen);
    case UsbCommand::GetData:
        return processUsbCommandGetData(rxBuf, rxLen, txBuf, txLen);
    case UsbCommand::GetDataFromSensor:
        return processUsbCommandGetDataFromSensor(rxBuf, rxLen, txBuf, txLen);
    default:
        return getUsbResponseCommand(usbCommand, UsbErrorCode::BadCommandError);
    }
}

#endif