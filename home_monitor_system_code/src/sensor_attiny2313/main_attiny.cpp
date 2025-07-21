#ifdef ATTINY2313A
extern "C"
{
    #include "avr/io.h"
    #include "avr/interrupt.h"
    #include "avr/sleep.h"
    #include "util/delay.h"
}

#include "nrf24.hpp"
#include "spi.hpp"
#include "packet/packet.hpp"
#include "platform/attiny2313/utils.hpp"
#include "platform/attiny2313/eeprom.hpp"
#include "platform/attiny2313/config.hpp"
#include "platform/attiny2313/led_status.hpp"
#include "platform/attiny2313/error_process.hpp"
#include "sensor/battery_voltage.hpp"
#include "sensor/bme_sensor.hpp"
// Attiny2313 -> sleep 18 uA

uint8_t rxAddress[5]; // Read from EEPROM!

enum class State : uint8_t
{
    // PwrUp,
    // PwrDown,
    ToIdle,
    ToIdleMaxRetr,
    Idle,
    ProcessReceivedData,
    DoMeasurements,
    SendPacket,
    ProcessIrq
};

volatile State state;


// void setDone()
// {
//     PORTD |= (1<<PD6);
//     _delay_ms(1);
//     PORTD &= ~(1<<PD6);
//     _delay_ms(1);
// }

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
        state = State::ToIdle;
        blinkLed(LedColor::Green, BlinkCount::Send);
    }
}

ISR(INT1_vect)
{
    state = State::ProcessIrq;
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
    
    eDELAY_MS(10);
    nrf24::flushRx();

    if(sensorPacket::checkCrc(packet.raw, dataCoutToProcess) == -1)
    {
        blinkLed(LedColor::Red, BlinkCount::WrongCRC);
        return 1;
    }

    if(packet.General.header.direction != sensorPacket::PacketDirection::Request)
        return 1;

    if(packet.General.header.errorFlag == sensorPacket::PacketError::Error)
        return 1;

    packet.Info.header.direction = sensorPacket::PacketDirection::Response;
    packet.Info.header.errorFlag = sensorPacket::PacketError::NoError;

    readBytesEEPORM(EepromMap::Identifier, packet.General.identifierRaw, 4);

    switch (packet.General.header.type)
    {
    case sensorPacket::PacketType::SensorInfo:
        readBytesEEPORM(EepromMap::SoftwareVersion, packet.Info.softwareVersionRaw, 4);
        readBytesEEPORM(EepromMap::HardwareVersion, packet.Info.hardwareVersionRaw, 4);
        bme::isIdValid(&packet.Info.sensorType);

        packetSize = 15;
        break;
    case sensorPacket::PacketType::SensorCalibData0:
        bme::BmeSelector<bmeType>::readCalibrationData0(packet.CalibData.calibData);
        packetSize = 32;
        break;
    case sensorPacket::PacketType::SensorCalibData1:
        bme::BmeSelector<bmeType>::readCalibrationData1(packet.CalibData.calibData);
        packetSize = 32;
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
    _delay_ms(20);
    
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
#elif
    state = State::DoMeasurements;
#endif

    while (true)
    {
        switch (state)
        {
        case State::ProcessIrq:
            processIrq();
            break;
        case State::ToIdleMaxRetr:
            nrf24::flushTx();
            // transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
            eDELAY_MS(10);
            [[fallthrough]];
        case State::ToIdle:
        {
            uint8_t resetReg = 0x70;
            transmitSPI(&resetReg, nullptr, nrf24::WriteRegister(nrf24::RegMap::Status), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
            // eDELAY_MS(100);
            
            nrf24::setToPRX();
            state = State::Idle;
        }
            break;
        case State::Idle:
            // Do nothing
            break;
        case State::DoMeasurements:
            for (size_t i = 0; i < 32; i++)
            {
                packet.raw[i] = 0;
            }
            

            doMeasurements();
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
            if(processReceivedData())
            {
                turnOnRedLed();
                state = State::Idle;
            }
            else
            {
                state = State::SendPacket;
            }
            break;
        case State::SendPacket:
            sensorPacket::generateCrc(packet.raw, packetSize);
            eDELAY_MS(100);
            nrf24::setToPTX();
            eDELAY_MS(10);
            nrf24::sendData(packet.raw, packetSize);
            state = State::Idle;
            break;
        }

#if defined(NO_SLEEP_TIMER)
        if(isButtonPressed(&PINB, buttonRightPin) && state == State::Idle)
        {
            state = State::DoMeasurements;
            blinkLed(LedColor::Green, BlinkCount::Some);
        }
#endif
    }
}

#endif