#if defined(RP2040)
#include <stdint.h>
#include "rp2040/wireless_communicator/wireless_communicator.hpp"
#include "utilities.hpp"
#include "rp2040/config.hpp"
#include "nRF24.hpp"
#include "nrf24_config.hpp"
#include "rp2040/led.hpp"
#include "spi.hpp"
#include "rp2040/logger/logger.hpp"

List<sensor::Sensor, maxKnownSensors> knownSensors;
RingBuffer<sensorPacket::SensorPacketWithLen, maxPacketToSend> sendPacketRingBuffer;
extern portType NRF24_CSN_PORT;
extern pinType NRF24_CSN_PIN;
volatile uint32_t isSending = 0;

void processSensorSends()
{
    static uint64_t lastSendTime = 0;

    if(isSending)
        return;
    if(sendPacketRingBuffer.getDataCount() == 0)
        return;

    if(getTime() - lastSendTime < 10000000)
        return;
    lastSendTime = getTime();
    
    // sleep_ms(5000);
    
    sensorPacket::SensorPacketWithLen packetToSend; 
    sendPacketRingBuffer.popData(packetToSend);
    
    sensorPacket::generateCrc(packetToSend.packet.raw, packetToSend.dataLen);
    // blinkLed(LedState::OnRed, packetToSend.dataLen, 500);
    
    // TODO in feature change nrf24 address
    nrf24::setToPTX();
    // transmitSPI(nullptr, nullptr, WriteCmd(Commands::FlushTx), 0, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    // sleep_ms(100);
    nrf24::sendData(packetToSend.packet.raw, packetToSend.dataLen);
    isSending = 1;
    log("Radio", LogLevel::Debug, "Send: %d bytes: 0x%s", packetToSend.dataLen, sensorPacket::packetToStr(&packetToSend.packet, packetToSend.dataLen).buf);
}

int processSensorPayload(const uint8_t* payload, uint8_t len)
{
    if(sensorPacket::checkCrc(payload, len) != 0)
        return -1;

    uint64_t recvTime = getTime();

    // Copying is better because payload has not const len
    sensorPacket::SensorPacket recv;
    for (uint32_t i = 0; i < len; i++)
        recv.raw[i] = payload[i];

    sensor::Sensor tmpSensor;
    sensor::Sensor* ptrSensor = &tmpSensor;

    // Looking for known identifier
    int isKnownSensor = knownSensors.valueByExpression([](sensor::Sensor& refSensor, void* args)->int {
        sensorPacket::SensorPacket* tmp = reinterpret_cast<sensorPacket::SensorPacket*>(args);
        if(refSensor.sensorInfo.identifier == tmp->General.identifierValue)
            return 0;
        return -1;
    }, ptrSensor, &recv);

    if(isKnownSensor == -1)
    {
        tmpSensor.sensorInfo.identifier = recv.General.identifierValue;
        if(knownSensors.add(tmpSensor) == -1)
        {
            // TODO if list is full
        }
        
        knownSensors.valueAt(0, ptrSensor);
        log("Sensor", LogLevel::Info, "Added new sensor with id %08X", tmpSensor.sensorInfo.identifier);
    }

    log("Packet", LogLevel::Debug, "Processing packet: %s", sensorPacket::packetToStrInfo(&recv, len).buf);
    log("Packet", LogLevel::Debug, "Packet: %s", sensorPacket::packetHeaderToStr(&recv.General.header).buf);
    switch (recv.General.header.type)
    {
    case sensorPacket::PacketType::Test:
        break;
    
    case sensorPacket::PacketType::SensorInfo:
        ptrSensor->sensorInfo.hardwareVersion = recv.Info.hardwareVersionValue;
        ptrSensor->sensorInfo.softwareVersion = recv.Info.softwareVersionValue;
        ptrSensor->sensorInfo.sensorType = static_cast<sensor::SensorType>(recv.Info.sensorType);
        ptrSensor->sensorInfo.initializationTime = recvTime;
        break;

    case sensorPacket::PacketType::SensorData0:
    {
        sensor::SensorData0 data {
            .packet = recv,
            .recvTime = recvTime
        };
        ptrSensor->sensorData.pushData(data);
        break;
    }
    case sensorPacket::PacketType::SensorCalibData0:
        for (uint32_t i = 0; i < 26; i++)
        {
            ptrSensor->sensorInfo.calibrationData[i] = recv.CalibData.calibData[i];
        }
        ptrSensor->sensorInfo.isCalibrationDataKnown = 1;
        break;
    } 

    ptrSensor->sensorInfo.lastRecvDataTime = recvTime;

    if(ptrSensor->sensorInfo.sensorType == sensor::SensorType::Unknown)
    {
        sensorPacket::SensorPacketWithLen packetToSend;
        for (size_t i = 0; i < 32; i++)
        {
            packetToSend.packet.raw[i] = 0;
        }
        
        packetToSend.packet.CalibData.header.direction = sensorPacket::PacketDirection::Request;
        packetToSend.packet.CalibData.header.errorFlag = sensorPacket::PacketError::NoError;
        packetToSend.packet.CalibData.header.type = sensorPacket::PacketType::SensorInfo;
        packetToSend.packet.CalibData.identifierValue = ptrSensor->sensorInfo.identifier;
        
        packetToSend.dataLen = 6;
        
        sendPacketRingBuffer.pushData(packetToSend);
        log("Sensor", LogLevel::Debug, "Unknown sensor with id 0x%08X, send request %d bytes: %s", tmpSensor.sensorInfo.identifier, packetToSend.dataLen, sensorPacket::packetToStr(&packetToSend.packet, packetToSend.dataLen).buf);
        return 0;
    }

    if(!ptrSensor->sensorInfo.isCalibrationDataKnown)
    {
        sensorPacket::SensorPacketWithLen packetToSend;
        for (size_t i = 0; i < 32; i++)
        {
            packetToSend.packet.raw[i] = 0;
        }
        
        packetToSend.packet.CalibData.header.direction = sensorPacket::PacketDirection::Request;
        packetToSend.packet.CalibData.header.errorFlag = sensorPacket::PacketError::NoError;
        packetToSend.packet.CalibData.header.type = sensorPacket::PacketType::SensorCalibData0;
        packetToSend.packet.CalibData.identifierValue = ptrSensor->sensorInfo.identifier;

        packetToSend.dataLen = 6;

        sendPacketRingBuffer.pushData(packetToSend);
        log("Sensor", LogLevel::Debug, 
            "Sensor 0x%08X, unknown calibration data , send request %d bytes: %s",
            ptrSensor->sensorInfo.identifier, 
            packetToSend.dataLen, 
            sensorPacket::packetToStr(&packetToSend.packet, packetToSend.dataLen).buf
        );
        return 0;
    }


    return 0;
}

// 0. rx_dr interrupt
// 1. Read payload
// 2. Clear rx_dr
// 3. Read fifo status
// 4. If more data execute step 1.
void processIrqStateNRF24()
{
    uint8_t dataToProcess[32];
    uint8_t dataCoutToProcess = 32;

    uint8_t status = nrf24::getStatusReg();
    
    readPayload:
    if(status & nrf24::SetRegister(nrf24::Reg::Status::rx_dr))
    {

        transmitSPI(&dataCoutToProcess, &dataCoutToProcess, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        
        if(dataCoutToProcess > 32)
        {
            transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
            log("Radio", LogLevel::Error, "Data > 32 !");
            return;
        }

        transmitSPI(dataToProcess, dataToProcess, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCoutToProcess, NRF24_CSN_PORT, NRF24_CSN_PIN);
        blinkLed(LedState::OnGreen, 2);
        
        log("Radio", LogLevel::Debug, "Recv: %d bytes: 0x%s", dataCoutToProcess, sensorPacket::packetToStr(dataToProcess, dataCoutToProcess).buf);

        if(processSensorPayload(dataToProcess, dataCoutToProcess) == -1)
            setLedState(LedState::OnRed);
        
        uint8_t resetReg = static_cast<uint8_t>(nrf24::Reg::Status::rx_dr);
        transmitSPI(&resetReg, &resetReg, nrf24::WriteRegister(nrf24::RegMap::Status), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::tx_ds))
    {
        isSending = 0;
        // Send data successfull
        nrf24::setToPRX();
        blinkLed(LedState::OnGreen, 3, 10);
        log("Radio", LogLevel::Debug, "Sending Done");
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::max_rt))
    {
        isSending = 0;
        transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
        
        // TODO Add error when can not send data
        blinkLed(LedState::OnRed, 5);
        setLedState(LedState::OnRed);
        nrf24::setToPRX();
        log("Radio", LogLevel::Error, "Send max retransmission");
    }
    
    uint8_t resetReg = 0x70;
    transmitSPI(&resetReg, &resetReg, nrf24::WriteRegister(nrf24::RegMap::Status), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);

    uint8_t fifoStatus = 0;
    transmitSPI(&fifoStatus, &fifoStatus, nrf24::ReadRegister(nrf24::RegMap::FifoStatus), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    if(!(fifoStatus & 1))
        goto readPayload;

    // NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
}

const uint8_t rxAddress[5] = {'a', 'b', 'c', 'd', '0'};
const uint8_t txAddress[5] = {'a', 'b', 'c', 'd', '0'};

void initWirelessCommunicator()
{
    log("Radio", LogLevel::Info, "Initializing...");
    if(!nrf24::isConnected())
    {
        log("Radio", LogLevel::Error, "Radio connection error!");
        while(true)
        {
            blinkLed(LedState::OnRed, 2, 300);
            eDELAY_MS(1000);
        }
    }
    nrf24::initnRF24(rxAddress, txAddress);
    nrf24::setToPRX();
    nrf24::flushRx();
    nrf24::flushTx();
    log("Radio", LogLevel::Info, "Initialized!");
}

#endif