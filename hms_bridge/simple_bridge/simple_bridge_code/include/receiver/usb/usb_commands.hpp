/// @file usb_commands.hpp
/// @brief Contain functions to process USB commands
/// @author bravePeer
/// @details 
/// @date 17-08-2025

#pragma once
#if defined(RP2040)
#include <stdint.h>
#include "platform/rp2040/wireless_communicator/wireless_communicator.hpp"
#include "sensor/sensor/sensor.hpp"
#include "utilities/utilities.hpp"
#include "receiver/usb/usb_packet.hpp"

/// @brief This function is used to describe args
/// @param rxBuf Raw rx buffer, includes command and length bytes
/// @param rxLen Length of rxBuf
/// @param txBuf Data to write to host not include command and length bytes
/// @param txLen Length of data to write to host not include command and lenght bytes
/// @return Return command error code
UsbCommandErrorCode processUsbGeneric(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandAvailableData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    uint16_t* availableData = reinterpret_cast<uint16_t*>(&txBuf[0]);
    *availableData = usbData.size;
    txLen = 2;
    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetKnownSensorsCount(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen > 2)
        return UsbCommandErrorCode::BadDataLenght;

    valueToBytes(sensor::knownSensors.size(), &txBuf[0]);
    txLen = 4;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetKnownSensors(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen > 3)
    {
        return UsbCommandErrorCode::BadDataLenght;
    }

    const uint32_t pageSize = 60;
    const uint32_t pageSensorCount = 15;
    uint8_t page = 0;

    if(rxLen == 3)
    {
        page = rxBuf[2];
    }

    sensor::Sensor* sensor;
    int sensorsCount = sensor::knownSensors.size();

    if(sensorsCount == 0)
        return UsbCommandErrorCode::NotKnownSensors;
    else if((sensorsCount - (pageSensorCount * page)) <= 0)
        return UsbCommandErrorCode::PageWithNoData;

    txLen = 0;
    uint32_t knownSensorIndex = pageSize * static_cast<uint32_t>(pageSensorCount);
    do
    {
        valueToBytes(sensor->info.identifier, &txBuf[txLen]);
        txLen += 4;
        if(txLen > 60)
        {
            txLen -= 4;
            break;
        }
    } while(sensor::knownSensors.valueAt(knownSensorIndex++, sensor) == 0);

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetSensorInfo(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = sensor::knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.info.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    valueToBytes(sensor->info.identifier, &txBuf[0]);
    valueToBytes(static_cast<uint32_t>(sensor->info.sensorType), &txBuf[4]);
    valueToBytes(sensor->info.softwareVersion, &txBuf[8]);
    valueToBytes(sensor->info.hardwareVersion, &txBuf[12]);   
     
    txBuf[16] = sensor->info.address[0];
    txBuf[17] = sensor->info.address[1];
    txBuf[18] = sensor->info.address[2];
    txBuf[19] = sensor->info.address[3];
    txBuf[20] = sensor->info.address[4];
    valueToBytes(sensor->info.initializationTime, &txBuf[21]);
    txLen = 29;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode GetSensorCalibData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(!(rxLen == 6 || rxLen == 8))
        return UsbCommandErrorCode::BadDataLenght;

    if(rxLen == 8)
        return UsbCommandErrorCode::FunctionNotSupported;

    
    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = sensor::knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.info.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    /// @todo Add checking calibration data and copying data
    // if(!sensor->info.isCalibrationDataKnown)
    //     return UsbCommandErrorCode::SensorCalibrationDataNotKnown;

    // txLen = 0;
    // for (uint8_t& a : sensor->info.calibrationData)
    // {
    //     txBuf[txLen] = a;
    //     txLen++;
    // }

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetSensorDataCount(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = sensor::knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.info.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    valueToBytes(sensor->lastRawDatas.size(), &txBuf[0]);
    txLen = 4;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetSensorData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = sensor::knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.info.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

        /// @todo Not finished
    // sensor::SensorData0 sensorData;
    // result = sensor->sensorData.pop(sensorData);

    // if(result == -1)
    //     return UsbCommandErrorCode::SensorNoDataAvailable;

    // valueToBytes(sensorData.recvTime, &txBuf[0]);
    // for (uint32_t i = 0; i < 32; i++)
    // {
    //     txBuf[8 + i] = sensorData.packet.raw[i];
    // }

    // txLen = 32 + 8;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetLastSensorData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen)
{
    if(rxLen != 6)
        return UsbCommandErrorCode::BadDataLenght;

    uint32_t sensorIdentifier = bytesToUint32(&rxBuf[2]);
    sensor::Sensor* sensor = nullptr;
    int result = sensor::knownSensors.valueByExpression([](sensor::Sensor& sensor, void* arg)->int {
        if(sensor.info.identifier == *reinterpret_cast<uint32_t*>(arg))
            return 0;
        return -1;
    }, sensor, &sensorIdentifier);

    if(result == -1)
        return UsbCommandErrorCode::SensorNotKnown;

    /// @todo Not finished
    // int dataInBuffer = sensor->sensorData.size();

    // if(dataInBuffer == -1)
    //     return UsbCommandErrorCode::SensorNoDataAvailable;

    // sensor::SensorData0 sensorData;
    // for(int i = 0; i < dataInBuffer; i++)
    // {
    //     sensor->sensorData.pop(sensorData);
    //     sensor->sensorData.push(sensorData);
    // }

    // valueToBytes(sensorData.recvTime, &txBuf[0]);
    // for (uint32_t i = 0; i < 32; i++)
    // {
    //     txBuf[8 + i] = sensorData.packet.raw[i];
    // }

    // txLen = 32 + 8;

    return UsbCommandErrorCode::NoError;
}

UsbCommandErrorCode processUsbCommandGetData(const uint8_t* rxBuf, uint16_t rxLen, uint8_t* txBuf, uint16_t& txLen) 
{
    memcpy(txBuf, usbData.data, usbData.size);
    txLen = usbData.size;
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
    case UsbCommand::GetSensorCalibData:
        errorCode = GetSensorCalibData(rxBuf, rxLen, txBuf, txLen);
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
    case UsbCommand::GetSensorLastData:
        errorCode = processUsbCommandGetLastSensorData(rxBuf, rxLen, txBuf, txLen);
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