#pragma once
#include <cstdint>
#include "receiver/logger/logger.hpp"
#include "packet.hpp"
#include "sensor/sensor/sensor.hpp"
#include "ring_buffer.hpp"
#include "list.hpp"
#include "utilities.hpp"
#include "nrf24.hpp"
#include "packetstr.hpp"
#include "general/platform_led.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "packet_processor/packet_processor.hpp"

// odpowiedzialne za wysyłanie danych do nrf i sprawdzanie pakietów

static constexpr int32_t maxPacketSendCount = 20;
static constexpr int32_t maxPacketRecvCount = 10;

inline RingBuffer<sensorPacket::SensorPacketWithLen, maxPacketSendCount> sendPacketRingBuffer;
inline RingBuffer<sensorPacket::SensorPacketWithLen, maxPacketRecvCount> recvPacketRingBuffer;
inline sensorPacket::SensorPacketWithLen sendingPacket;

static constexpr uint8_t rxAddress[5] = {'a', 'b', 'c', 'd', '0'};
static constexpr uint8_t txAddress[5] = {'a', 'b', 'c', 'd', '0'};

inline volatile uint32_t isSending = 0;

/// @brief Initialize NRF24
/// @return If 0 succsess otherwise -1
inline ssize_t initRadio()
{
    log("Radio", LogLevel::Info, "Initializing...");
    if(!nrf24::isConnected())
    {
        nrf24::resetStatusReg();
        vTaskDelay(pdMS_TO_TICKS(10));
        if(!nrf24::isConnected())
        {
            log("Radio", LogLevel::Error, "Radio connection error!");
            return -1;
        }
    }
    nrf24::initnRF24(rxAddress, txAddress);
    nrf24::setToPRX();
    log("Radio", LogLevel::Info, "Initialized!");

    return 0;
}

inline void processSensorSends(QueueHandle_t sendQueue)
{
    // static uint64_t lastSendTime = 0;

    // if(isSending)
    //     return;
    // if(sendPacketRingBuffer.size() == 0)
    //     return;

    // // if(getTime() - lastSendTime < 10000000)
    // // if(getTime() - lastSendTime < 5000)
    // if(getTime() - lastSendTime < 2000)
    //     return;
    // lastSendTime = getTime();
    
    // // delayMs(5000);
    
    // sensorPacket::SensorPacketWithLen packetToSend; 
    // sendPacketRingBuffer.pop(packetToSend);
    // sendingPacket = packetToSend;
    PacketToProcess packetToSend;

    if(xQueueReceive(sendQueue, &packetToSend, 0) == pdFALSE)
        return;

    

    // blinkLed(LedState::OnRed, packetToSend.dataLen, 500);
    
    // TODO in feature change nrf24 address
    // isSending = 1;
    log("Radio", LogLevel::Debug, "Sending: %d bytes: 0x%s", packetToSend.sendPacket.radioPacket.size, sensorPacket::packetToStr(&packetToSend.sendPacket.radioPacket.packet,packetToSend.sendPacket.radioPacket.size).buf);
    log("Radio", LogLevel::Debug, "Sending to: %lx, Header: %s", packetToSend.sendPacket.radioPacket.packet.General.identifierValue, sensorPacket::packetHeaderToStr(&packetToSend.sendPacket.radioPacket.packet.General.header).buf);
    
    nrf24::setToPTX();
    nrf24::sendData(packetToSend.sendPacket.radioPacket.packet.raw, packetToSend.sendPacket.radioPacket.size);
}

inline void processAfterSend()
{
    sensor::processSendPayload(sendingPacket);
    sendingPacket.size = 0;
}

/// @brief Check packet structure
/// @return 
inline ssize_t __attribute__((deprecated)) processPacketRecv()
{
    sensorPacket::SensorPacketWithLen packet;
    
    if(recvPacketRingBuffer.pop(packet) == -1)
        return 0;
    
    if(sensorPacket::checkCrc(packet.packet.raw, packet.size) == -1)
    {
        log("Radio", LogLevel::Error, "Crc error! %s", sensorPacket::packetToStr(packet.packet.raw, packet.size).buf);
        return -1;
    }
    
    log("Radio", LogLevel::Debug, "Recv header: %s", sensorPacket::packetHeaderToStr(&packet.packet.General.header).buf);
    
    sensor::processRecvPayload(packet);

    return 0;
}

inline void preparePacketToSend()
{
    auto packets = sensor::prepareSendPayload();
    sensorPacket::SensorPacketWithLen packet;
    
    while(packets.pop(packet) == 0)
    {
        sensorPacket::generateCrc(packet.packet.raw, packet.size);

        sendPacketRingBuffer.push(packet);
    }
}


// 0. rx_dr interrupt
// 1. Read payload
// 2. Clear rx_dr
// 3. Read fifo status
// 4. If more data execute step 1.
inline void processIrqStateNRF24(QueueHandle_t packetToProcessQueue)
{
    uint8_t dataToProcess[32];
    uint8_t dataCoutToProcess = 0;

    uint8_t status = nrf24::getStatusReg();
    
    readPayload:
    if(status & nrf24::SetRegister(nrf24::Reg::Status::rx_dr))
    {
        transmitSpiNrf24(&dataCoutToProcess, &dataCoutToProcess, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1);
        
        if(dataCoutToProcess > 32)
        {
            nrf24::flushRx();
            log("Radio", LogLevel::Error, "Data > 32 !");
            return;
        }

        transmitSpiNrf24(dataToProcess, dataToProcess, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCoutToProcess);
        // blinkLed(LedState::OnGreen, 2);
        
        log("Radio", LogLevel::Debug, "Recv: %d bytes: 0x%s", dataCoutToProcess, sensorPacket::packetToStr(dataToProcess, dataCoutToProcess).buf);

        // if(processRecvPayload(dataToProcess, dataCoutToProcess) == -1)
        //     setLedState(LedState::OnRed);
        PacketToProcess packetToProcess;
        packetToProcess.packetType = PacketToProcessType::RecvPacket;
        std::memcpy(packetToProcess.recvPacket.radioPacket.packet.raw, dataToProcess, dataCoutToProcess);
        packetToProcess.recvPacket.radioPacket.size = dataCoutToProcess;
        packetToProcess.recvPacket.recvTime = getTime();
        xQueueSend(packetToProcessQueue, &packetToProcess, 0);

        // sensorPacket::SensorPacketWithLen packetToProcess;
        // std::memcpy(packetToProcess.packet.raw, dataToProcess, dataCoutToProcess);
        // packetToProcess.size = dataCoutToProcess;
        // recvPacketRingBuffer.push(packetToProcess);
        
        uint8_t resetReg = static_cast<uint8_t>(nrf24::Reg::Status::rx_dr);
        transmitSpiNrf24(&resetReg, &resetReg, nrf24::WriteRegister(nrf24::RegMap::Status), 1);
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::tx_ds))
    {
        isSending = 0;
        // Send data successfull
        nrf24::setToPRX();
        // blinkLed(LedState::OnGreen, 3, 10);
        log("Radio", LogLevel::Debug, "Sending Done");
        //processAfterSend();
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::max_rt))
    {
        isSending = 0;
        nrf24::flushTx();
        nrf24::setToPRX();

        // TODO Add error when can not send data
        // blinkLed(LedState::OnRed, 5);
        // setLedState(LedState::OnRed);
        log("Radio", LogLevel::Error, "Send max retransmission");
        sendingPacket.size = 0;
    }
    
    uint8_t resetReg = 0x70;
    transmitSpiNrf24(&resetReg, &resetReg, nrf24::WriteRegister(nrf24::RegMap::Status), 1);

    uint8_t fifoStatus = 0;
    transmitSpiNrf24(&fifoStatus, &fifoStatus, nrf24::ReadRegister(nrf24::RegMap::FifoStatus), 1);
    if(!(fifoStatus & 1))
        goto readPayload;
}