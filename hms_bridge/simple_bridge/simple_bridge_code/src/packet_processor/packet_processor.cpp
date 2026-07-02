#include "packet_processor/packet_processor.hpp"
#include "sensor/sensor/sensor.hpp"
#include "sensor/sensor/sensor_utils.hpp"
#include "radio/radio.hpp"
#include "file_storage/file_storage.hpp"
#include "http_client/http_client_task.hpp"

const char* logTag = "PacketProcessorTask";

void processRecvPayload(PacketProcessorTaskArg* taskArg, PacketToProcess* packet)
{
    PacketToProcess requestPacket;

    ESP_LOGI(logTag, "Stack in recv fun: %d bytes", uxTaskGetStackHighWaterMark(NULL));
    const sensorPacket::SensorPacketWithLen* radioPacket = &packet->recvPacket.radioPacket;
    if(sensorPacket::checkCrc(radioPacket->packet.raw, radioPacket->size))
    {
        log("PacketProcessor", LogLevel::Error, "CRC error!");
        return;
    }

    ESP_LOGI(logTag, "Stack after CRC: %d bytes", uxTaskGetStackHighWaterMark(NULL));

    sensor::Sensor* sensor = nullptr;
    if(sensor::isSensorKnown(radioPacket->packet.General.identifierValue, &sensor, &sensor::knownSensors) == -1)
    {
        log("PacketProcessor", LogLevel::Warning, "Received packet from unknown sensor 0x%lx, adding to known sensors", radioPacket->packet.General.identifierValue);

        if(sensor::addSensor(radioPacket->packet.General.identifierValue, &sensor, &sensor::knownSensors) == -1)
        {
            log("PacketProcessor", LogLevel::Error, "Adding new sensor failed, list full?");
            return;
        }
    }
    else
    {
        log("PacketProcessor", LogLevel::Info, "Sensor known, identifier: 0x%lx", sensor->info.identifier);
    }
    ESP_LOGI(logTag, "Stack after is sensor known: %d bytes", uxTaskGetStackHighWaterMark(NULL));

    assert(sensor != nullptr);

    if(sensor->status.statusReg.isSensorInfoKnown == 0 
        && radioPacket->packet.General.header.type != sensorPacket::PacketType::SensorInfo)
    {
        log("PacketProcessor", LogLevel::Warning, "First packet from sensor is not SensorInfo, requesting info, id: 0x%lx", radioPacket->packet.General.identifierValue);
        requestPacket.packetType = PacketToProcessType::SendPacket;
        requestPacket.sendPacket.radioPacket.packet.General.header.direction = sensorPacket::PacketDirection::Request;
        requestPacket.sendPacket.radioPacket.packet.General.header.errorFlag = sensorPacket::PacketError::NoError;
        requestPacket.sendPacket.radioPacket.packet.General.header.type = sensorPacket::PacketType::SensorInfo;
        requestPacket.sendPacket.radioPacket.packet.General.identifierValue = radioPacket->packet.General.identifierValue;
        requestPacket.sendPacket.radioPacket.size = 6;
        QueueHandle_t tmp = taskArg->packetToProcessQueue;
        if(xQueueSend(tmp, &requestPacket, 0) != pdTRUE)
        {
            log("PacketProcessor", LogLevel::Error, "Queue full, can not request sensor info, id: 0x%lx", radioPacket->packet.General.identifierValue);
            return;
        }
        return;
    }

    int val = sensor::processRecvPacket(packet->recvPacket.radioPacket, sensor, &sensor::knownSensors);

    log("PacketProcessor", LogLevel::Debug, "Processed received packet from sensor 0x%lx, result: %d", radioPacket->packet.General.identifierValue, val);
    sensor::dumpSensorInfo(sensor);

    // Check if bridge need sensor configuration
    if(sensor->status.statusReg.isSensorCalibReceived == 0 
        && sensor->status.statusReg.isSensorInfoKnown == 1
        && sensor->status.statusReg.isSensorCalibReqSend == 0)
    {
        sensor->status.statusReg.isSensorCalibReqSend = 1;
        log("PacketProcessor", LogLevel::Info, "Requesting calib data, id: 0x%lx", sensor->info.identifier);
        for (uint8_t i = 0; i < sensor->info.calibrationPacketsMaxCount; i++)
        {
            requestPacket.packetType = PacketToProcessType::SendPacket;
            requestPacket.sendPacket.radioPacket.packet.General.header.direction = sensorPacket::PacketDirection::Request;
            requestPacket.sendPacket.radioPacket.packet.General.header.errorFlag = sensorPacket::PacketError::NoError;
            requestPacket.sendPacket.radioPacket.packet.General.header.type = static_cast<sensorPacket::PacketType>(std::to_underlying(sensorPacket::PacketType::SensorCalibData0) + i);
            requestPacket.sendPacket.radioPacket.packet.General.identifierValue = sensor->info.identifier;
            requestPacket.sendPacket.radioPacket.size = 6;
            QueueHandle_t tmp = taskArg->packetToProcessQueue;
            if(xQueueSend(tmp, &requestPacket, 0) != pdTRUE)
            {
                log("PacketProcessor", LogLevel::Error, "Queue full, can not request sensor calibration data, id: 0x%lx", sensor->info.identifier);
                return;
            }
        }
        return;
    }

    if((sensor->status.statusReg.isSensorCalibReceived == 1
        && sensor->status.statusReg.isSensorInfoKnown == 1
        && sensor->status.statusReg.isSensorCalibReqSend == 1))
    {
        log("PacketProcessor", LogLevel::Info, "Sensor calib data known");
        sensor::dumpSensorCalibrationData(sensor);
    }
        
    if((sensor->status.statusReg.isSensorCalibReceived == 1
        && sensor->status.statusReg.isSensorInfoKnown == 1
        && sensor->status.statusReg.isSensorCalibReqSend == 1
        && sensor->status.statusReg.isSensorSaved == 0))
    {
        sensor->status.statusReg.isSensorSaved = 1;
        log("PacketProcessor", LogLevel::Info, "Send sensor info to storage, id: 0x%lx", sensor->info.identifier);

        FileStorageQueueData data;
        data.dataType = FileStorageQueueDataType::StoreSensor;
        data.sensorId = sensor->info.identifier;
        data.sensorPtr = sensor;
        if(xQueueSend(taskArg->fileStorageQueue, &data, 0) != pdTRUE)
        {
            log("PacketProcessor", LogLevel::Error, "Queue full, can not send sensor to storage, id: 0x%lx", sensor->info.identifier);
            return;
        }
    }

    // Check if bridge need sensor data
    if(sensor->status.statusReg.isSensorDataReceived == 0 
        && sensor->status.statusReg.isSensorInfoKnown == 1
        && sensor->status.statusReg.isSensorDataReqSend == 0)
    {
        log("PacketProcessor", LogLevel::Info, "Requesting data, id: 0x%lx", sensor->info.identifier);
        for (uint8_t i = 0; i < sensor->info.dataPacketsMaxCount; i++)
        {
            requestPacket.packetType = PacketToProcessType::SendPacket;
            requestPacket.sendPacket.radioPacket.packet.General.header.direction = sensorPacket::PacketDirection::Request;
            requestPacket.sendPacket.radioPacket.packet.General.header.errorFlag = sensorPacket::PacketError::NoError;
            requestPacket.sendPacket.radioPacket.packet.General.header.type = static_cast<sensorPacket::PacketType>(std::to_underlying(sensorPacket::PacketType::SensorData0) + i);
            requestPacket.sendPacket.radioPacket.packet.General.identifierValue = sensor->info.identifier;
            requestPacket.sendPacket.radioPacket.size = 6;
            QueueHandle_t tmp = taskArg->packetToProcessQueue;
            if(xQueueSend(tmp, &requestPacket, 0) != pdTRUE)
            {
                log("PacketProcessor", LogLevel::Error, "Queue full, can not request sensor data, id: 0x%lx", sensor->info.identifier);
                return;
            }
            sensor->status.setSendDataReqPackets(i); // Set bit to request this packet
        }
        return;
    }

    if(sensor->status.statusReg.isSensorInfoKnown == 1
        && sensor->status.statusReg.isSensorCalibReceived == 1
        && sensor->status.statusReg.isSensorDataReqSend == 1
        && sensor->status.statusReg.isSensorDataReceived == 0)
    {
        log("PacketProcessor", LogLevel::Info, "Sensor id: 0x%lx, not all data received, waiting", sensor->info.identifier);
    }

    if(sensor->status.statusReg.isSensorInfoKnown == 1
        && sensor->status.statusReg.isSensorCalibReceived == 1
        && sensor->status.statusReg.isSensorDataReqSend == 1
        && sensor->status.statusReg.isSensorDataReceived == 1) // After process last data packet this bit will be set
    {
        log("PacketProcessor", LogLevel::Info, "Sensor id: 0x%lx, all data received", sensor->info.identifier);
        // sensor::dumpSensorData(sensor);

        sensor->status.resetSendDataReqPackets();
        sensor->calculateData();
        HttpClientTaskData httpData;
        httpData.command = HttpClientTaskCommand::SendSensorData;
        httpData.sensorData.sensorId = sensor->info.identifier;
        httpData.sensorData.sensorType = static_cast<uint32_t>(sensor->info.sensorType);
        sendToHttpClientTask(&httpData);

        FileStorageQueueData data;
        data.dataType = FileStorageQueueDataType::StoreResult;
        data.sensorId = sensor->info.identifier;
        data.sensorPtr = sensor;
        if(xQueueSend(taskArg->fileStorageQueue, &data, 0) != pdTRUE)
        {
            log("PacketProcessor", LogLevel::Error, "Queue full, can not send calculated data to storage, id: 0x%lx", sensor->info.identifier);
            return;
        }
    }
}

void processSendPayload(PacketProcessorTaskArg* taskArg, PacketToProcess* packet)
{
    ESP_LOGI(logTag, "Before CRC: %x", packet->sendPacket.radioPacket.packet.General.crc);
    sensorPacket::generateCrc(packet->sendPacket.radioPacket);
    ESP_LOGI(logTag, "After CRC: %x", packet->sendPacket.radioPacket.packet.General.crc);

    if(xQueueSend(taskArg->toRadioTaskQueue, packet, 0) != pdTRUE)
    {
        log("PacketProcessor", LogLevel::Error, "Queue full, can not send packet to radio task");
        return;
    }

    notifyRadioTask(RadioTaskNotifyBits::PacketToSent);
}

void packetProcessorTask(void* arg)
{
    auto taskArg = static_cast<PacketProcessorTaskArg*>(arg);

    assert(taskArg->packetToProcessQueue != nullptr);
    assert(taskArg->toRadioTaskQueue != nullptr);
    ESP_LOGI(logTag, "Started!");

    while (true)
    {
        PacketToProcess packet;
        if(xQueueReceive(taskArg->packetToProcessQueue, &packet, portMAX_DELAY) == pdFALSE)
        {
            continue;
        }
        
        ESP_LOGV(logTag, "Stack after queue recv: %d bytes", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGI(logTag, "Processing packet of type %ld", static_cast<int32_t>(packet.packetType));
        // Validate CRC
        if(packet.packetType == PacketToProcessType::RecvPacket)
        {
            processRecvPayload(taskArg, &packet);
        }
        else if(packet.packetType == PacketToProcessType::SendPacket)
        {
            processSendPayload(taskArg, &packet);
        }
        else
        {
            log(logTag, LogLevel::Warning, "Unknown packet type to process");
        }
        portYIELD();
    }
}