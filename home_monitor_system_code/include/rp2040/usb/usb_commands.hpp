#pragma once
#if defined(RP2040)
#include <stdint.h>
#include "rp2040/wireless_communicator/wireless_communicator.hpp"
#include "sensor/sensor_data.hpp"

/// @brief This function is used to describe args
/// @param rxBuf Raw rx buffer, includes command and length bytes
/// @param rxLen Length of rxBuf
/// @param txBuf Data to write to host not include command and length bytes
/// @param txLen Length of data to write to host not include command and lenght bytes
/// @return Return command byte to write to host
UsbCommandErrorCode processUsbGeneric(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    return UsbCommandErrorCode::NoError;
}


constexpr void valueToBytes(const int val, uint8_t* bytes)
{
    bytes[0] = val & 0xff;
    bytes[1] = (val >> 8) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 24) & 0xff;
}


constexpr void valueToBytes(const uint32_t val, uint8_t* bytes)
{
    bytes[0] = val & 0xff;
    bytes[1] = (val >> 8) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 24) & 0xff;
}

constexpr void valueToBytes(const uint64_t val, uint8_t* bytes)
{
    bytes[0] = val & 0xff;
    bytes[1] = (val >> 8) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 24) & 0xff;
    bytes[4] = (val >> 32) & 0xff;
    bytes[5] = (val >> 40) & 0xff;
    bytes[6] = (val >> 48) & 0xff;
    bytes[7] = (val >> 56) & 0xff;
}

constexpr uint32_t bytesToUint32(const uint8_t* bytes)
{
    uint32_t val = bytes[3];
    val <<= 8;
    val |= bytes[2];
    val <<= 8;
    val |= bytes[1];
    val <<= 8;
    val |= bytes[0];
    return val;
}


UsbCommandErrorCode processUsbCommandAvailableData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    uint16_t* availableData = reinterpret_cast<uint16_t*>(&txBuf[0]);
    *availableData = usbData.dataLen;
    txLen = 2;
    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetKnownSensorsCount(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen > 2)
        return UsbCommandErrorCode::BadDataLenght;

    valueToBytes(knownSensors.size(), &txBuf[0]);
    txLen = 4;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetKnownSensors(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen > 2)
        return UsbCommandErrorCode::FunctionNotSupported;

    sensor::Sensor* sensor;
    if(knownSensors.valueAt(0, sensor) == -1)
        return UsbCommandErrorCode::NotKnownSensors;

    uint32_t txBufIndex = 0;
    uint32_t knownSensorIndex = 1;
    do 
    {
        valueToBytes(sensor->sensorInfo.identifier, &txBuf[txBufIndex]);
        txBufIndex += 4;
        if(txBufIndex > 62)
        {
            txBufIndex -= 4;
            break;
        }
    } while(knownSensors.valueAt(knownSensorIndex++, sensor) == 0);

    txLen = txBufIndex;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetSensorInfo(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.sensorInfo.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    valueToBytes(sensor->sensorInfo.identifier, &txBuf[0]);
    txBuf[4] = static_cast<uint8_t>(sensor->sensorInfo.sensorType);
    valueToBytes(sensor->sensorInfo.softwareVersion, &txBuf[5]);
    valueToBytes(sensor->sensorInfo.hardwareVersion, &txBuf[9]);    
    txBuf[10] = sensor->sensorInfo.address[0];
    txBuf[11] = sensor->sensorInfo.address[1];
    txBuf[12] = sensor->sensorInfo.address[2];
    txBuf[13] = sensor->sensorInfo.address[3];
    txBuf[14] = sensor->sensorInfo.address[4];
    valueToBytes(sensor->sensorInfo.initializationTime, &txBuf[15]);
    valueToBytes(sensor->sensorInfo.lastRecvDataTime, &txBuf[23]);    
    txLen = 34;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetSensorDataCount(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.sensorInfo.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    valueToBytes(sensor->sensorData.getDataCount(), &txBuf[0]);
    txLen = 4;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetSensorData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.sensorInfo.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    sensor::SensorData sensorData;
    result = sensor->sensorData.popData(sensorData);

    if(result == -1)
        return UsbCommandErrorCode::SensorNoDataAvailable;

    valueToBytes(sensorData.recvTime, &txBuf[0]);
    for (uint32_t i = 0; i < 32; i++)
    {
        txBuf[8 + i] = sensorData.packet.raw[i];
    }

    txLen = 32 + 8;

    return UsbCommandErrorCode::NoError;
}


UsbCommandErrorCode processUsbCommandGetData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen) 
{
    memcpy(txBuf, usbData.data, usbData.dataLen);
    txLen = usbData.dataLen;
    return UsbCommandErrorCode::NoError;
}

/// @brief Executes proper commands wrote from host
/// @param rxBuf pointer to received packet
/// @param rxLen lenght of received packet
/// @param txBuf pointer to data to write
/// @param txLen reference to lenght of data to write
/// @return first byte of data to write to Host
uint8_t processUsbCommands(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    UsbCommand usbCommand = getUsbCommand(rxBuf[0]);
    UsbCommandErrorCode errorCode;
    switch (usbCommand)
    {
    case UsbCommand::AvailableData:
        errorCode = processUsbCommandAvailableData(rxBuf, rxLen, txBuf, txLen);
        break;
    case UsbCommand::GetKnownSensorsCount:
        errorCode = processUsbCommandGetKnownSensorsCount(rxBuf, rxLen, txBuf, txLen);
        break;
    case UsbCommand::GetKnownSensors:
        errorCode = processUsbCommandGetKnownSensors(rxBuf, rxLen, txBuf, txLen);
        break;
    case UsbCommand::GetSensorInfo:
        errorCode = processUsbCommandGetSensorInfo(rxBuf, rxLen, txBuf, txLen);
        break;
    case UsbCommand::GetSensorDataCount:
        errorCode = processUsbCommandGetSensorDataCount(rxBuf, rxLen, txBuf, txLen);
        break;
    case UsbCommand::GetSensorData:
        errorCode = processUsbCommandGetSensorData(rxBuf, rxLen, txBuf, txLen);
        break;
    case UsbCommand::GetData:
        errorCode = processUsbCommandGetData(rxBuf, rxLen, txBuf, txLen);
        break;
    default:
        return getUsbResponseCommand(usbCommand, UsbErrorCode::BadCommandError);
    }

    if(errorCode != UsbCommandErrorCode::NoError)
    {
        valueToBytes(static_cast<uint32_t>(errorCode), &txBuf[0]);
        txLen = 4;
        return getUsbResponseCommand(usbCommand, UsbErrorCode::InternalCommandError);
    }

    return getRawUsbCommand(usbCommand);
}

#endif