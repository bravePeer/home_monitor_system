extern "C"
{
    #include "avr/io.h"
    #include "avr/interrupt.h"
    #include "avr/sleep.h"
    #include "util/delay.h"
    #include "avr/wdt.h"
}

#include "nrf24.hpp"
#include "spi.hpp"
#include "utils.hpp"
#include <packet.hpp>
#include "eeprom.hpp"
#include "config.hpp"
#include "led_status.hpp"
#include "error_process.hpp"
#include "sensor_features/battery_voltage.hpp"
#include "sensor_features/bme_sensor.hpp"
#include "sensor_features/work_done.hpp"
// Attiny2313 -> sleep 18 uA

#include "platform_time.hpp"
#include "state.hpp"
#include "sleep.hpp"

uint8_t rxAddress[5]; // Read from EEPROM!


volatile uint8_t workDoneFlag = 0;

void processIrq()
{
    uint8_t status = nrf24::getStatusReg();
    if(status & nrf24::SetRegister(nrf24::Reg::Status::rx_dr))
    {
        state = State::ProcessReceivedData;
        blinkLed(LedColor::Green, BlinkCount::Received);
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::max_rt))
    {
        state = State::ToIdleMaxRetr;
        blinkLed(LedColor::Red, BlinkCount::MaxRetr);
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::tx_ds))
    {
        if(workDoneFlag)
        {   
            /// @todo Set work done
            // state = State::WorkDone;
        }
        else
            state = State::ToIdle;
        blinkLed(LedColor::Green, BlinkCount::Send);
    }
}


/// @brief Interrupt on nRF24 IRQ pin
ISR(INT1_vect)
{
    state = State::ProcessIrq;
    shouldWait = 1;
}



uint8_t packetSize = 32;
sensorPacket::SensorPacket packet;

// 0. rx_dr interrupt
// 1. Read payload
// 2. Clear rx_dr
// 3. Read fifo status
// 4. If more data execute step 1.
uint8_t processReceivedData()
{
    uint8_t dataCoutToProcess = nrf24::getRxPayloadCount();        
    if(dataCoutToProcess > 32)
    {
        nrf24::flushRx();
        return 1;
    }
    nrf24::readRxPayloadRaw(packet.raw, dataCoutToProcess);

    // 2. Clear rx_dr
    uint8_t resetReg = 0x40;
    transmitSPI(&resetReg, &resetReg, nrf24::WriteRegister(nrf24::RegMap::Status), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    
    // delayMs(10); /// @TODO NWM idk if needed
    nrf24::flushRx();

    if(sensorPacket::checkCrc(packet.raw, dataCoutToProcess) == -1)
    {
        blinkLed(LedColor::Red, BlinkCount::WrongCRC);
        return 2;
    }

    if(packet.General.header.direction != sensorPacket::PacketDirection::Request)
        return 3;

    if(packet.General.header.errorFlag == sensorPacket::PacketError::Error)
        return 4;

    packet.Info.header.direction = sensorPacket::PacketDirection::Response;
    packet.Info.header.errorFlag = sensorPacket::PacketError::NoError;

    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t idTmp;
        readBytesEEPORM(static_cast<uint8_t>(EepromMap::Identifier) + i, &idTmp, 1);
        if(idTmp != packet.General.identifierRaw[i])
            return 5; // Wrong identifier
    }

     // switchLabel:
    switch (packet.General.header.type)
    {
    case sensorPacket::PacketType::SensorInfo:
        readBytesEEPORM(EepromMap::SoftwareVersion, &packet.raw[6], 14); // Read all info from EEPROM
        // readBytesEEPORM(EepromMap::SoftwareVersion, packet.Info.softwareVersionRaw, 4);
        // readBytesEEPORM(EepromMap::HardwareVersion, packet.Info.hardwareVersionRaw, 4);
        // readBytesEEPORM(EepromMap::SensorType, packet.Info.sensorTypeRaw, 4);
        // readBytesEEPORM(EepromMap::DataPacketsMaxCount, packet.Info.dataPacketsMaxCount, 1);
        // readBytesEEPORM(EepromMap::CalibrationPacketsMaxCount, packet.Info.calibrationPacketsMaxCount, 1);
        bme::isIdValid(&packet.Info.sensorTypeRaw[0]);

        packetSize = 20;
        break;
    case sensorPacket::PacketType::SensorCalibData0:
        bme::BmeSelector<bmeType>::readCalibrationData0(packet.CalibData.calibData);
        packetSize = 32;
        break;
    case sensorPacket::PacketType::SensorCalibData1:
        bme::BmeSelector<bmeType>::readCalibrationData1(packet.CalibData.calibData);
        readBytesEEPORM(EepromMap::ResistorR1Value, &packet.CalibData.calibData[16], 4);
        packetSize = 26;
        break;
    case sensorPacket::PacketType::CmdWorkDone:
        packet.General.header.type = sensorPacket::PacketType::ResponseOK;
        packetSize = 6;
        workDoneFlag = 1;
        break;
    default:
        packet.Info.header.errorFlag = sensorPacket::PacketError::Error;
        break;
    } 
    return 0;
}

int main()
{
    init();

    turnOnRedLed();
    readBytesEEPORM(EepromMap::RxAddress, rxAddress, 5);

    intSPI(9600);
    
    // nrf24::resetStatusReg();
    if(!nrf24::isConnected())
       errorState(BlinkCount::Nrf24Error);
    
    if(!bme::BmeSelector<bmeType>::isValid())
        errorState(BlinkCount::BmeError);

    nrf24::initnRF24(rxAddress, rxAddress);
    nrf24::setToPRX();

    uint8_t dataBuf[8]{0};

    initMeasurementSensor();
    
    blinkLed(LedColor::Green, BlinkCount::Initialized);
    sei();

#if defined(NO_SLEEP_TIMER)
    state = State::Idle; // TODO In future auto measuring will be processing
#else
    state = State::DoMeasurements;
#endif

    uint8_t resetReg;

    while (true)
    {
        switch (state)
        {
        case State::ProcessIrq:
            processIrq();
            break;
        case State::ToIdleMaxRetr:
            nrf24::flushTx();
            [[fallthrough]];
        case State::ToIdle:
            resetReg = 0x70;
            transmitSPI(&resetReg, nullptr, nrf24::WriteRegister(nrf24::RegMap::Status), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
            // delayMs(100);
            nrf24::setToPRX();
            state = State::Idle;
            break;
        case State::Idle:
            sleep(40 * 2); // ~10s * 2
            if(shouldWait == 0)
                state = State::WorkDone;
            shouldWait = 0;
            break;
        case State::DoMeasurements:
            doMeasurements();
            // bme::BmeSelector<bmeType>::readAllData(&(packet.raw[6]));
            bme::BmeSelector<bmeType>::readAllData(dataBuf);
            packet.Data.header.direction = sensorPacket::PacketDirection::Report;
            packet.Data.header.errorFlag = sensorPacket::PacketError::NoError;
            packet.Data.header.type = sensorPacket::PacketType::SensorData0;
            
            readBytesEEPORM(EepromMap::Identifier, packet.Data.identifierRaw, 4);

            packet.Data.pressureRaw[2] = dataBuf[0];
            packet.Data.pressureRaw[1] = dataBuf[1];
            packet.Data.pressureRaw[0] = dataBuf[2];
            
            packet.Data.temperatureRaw[2] = dataBuf[3];
            packet.Data.temperatureRaw[1] = dataBuf[4];
            packet.Data.temperatureRaw[0] = dataBuf[5];
            
            packet.Data.humidityRaw[0] = dataBuf[7];
            packet.Data.humidityRaw[1] = dataBuf[6];
            readBatteryVoltage(packet.Data.batteryVoltageRaw);

            // 1B Header
            // 1B CRC
            // 4B identifier
            // 4B temperature
            // 4B pressure
            // 4B humidity
            // 4B battery voltage
            packetSize = 22;
            state = State::SendPacket;
            break;
        case State::ProcessReceivedData:
            if(uint8_t i = processReceivedData())
            {
                blinkLed(LedColor::Red, i);
                state = State::ToIdle;
            }
            else
            {
                state = State::SendPacket;
            }
            break;
        case State::SendPacket:
            sensorPacket::generateCrc(packet.raw, packetSize);
            // delayMs(100);
            nrf24::setToPTX();
            // delayMs(10);
            nrf24::sendData(packet.raw, packetSize);
            state = State::Idle;
            break;
        case State::WorkDone:
            blinkLed(LedColor::Green, BlinkCount::WorkDone);
            // indicateWorkDone();
            // setWorkDone();
            // state = State::Sleep;

            // nRF24 IRQ should not occur here because nRF24 is powered down
            nrf24::setPower(0);
            sleep(maxSleepCounter);
            state = State::WakeUp;
            break;
        case State::WakeUp:
            nrf24::setPower(1);
            state = State::DoMeasurements;
            sleep(1); // Wait for nRF24 to power up
            break;
        }

#if defined(ENABLE_BUTTON_MEASUREMENT)
        if(isButtonPressed(&PINB, buttonRightPin) && state == State::Idle)
        {
            state = State::DoMeasurements;
            blinkLed(LedColor::Green, BlinkCount::Some);
        }
#endif
    }
}