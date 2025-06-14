#ifdef ATTINY2313A
extern "C"
{
    #include "avr/io.h"
    #include "avr/interrupt.h"
    #include "avr/sleep.h"
    #include "util/delay.h"
}

#include "nrf24.hpp"
#include "bme280.hpp"
#include "spi.hpp"
#include "sensor/sensor_packet.hpp"
#include "attiny2313/utils.hpp"
#include "attiny2313/eeprom.hpp"
#include "attiny2313/config.hpp"
#include "attiny2313/led_status.hpp"
// Attiny2313 -> sleep 18 uA
// Read fueses
// avrdude.exe -p ATtiny2313A -c usbasp -U hfuse:r:-:h -U lfuse:r:-:h

// Write fuses
// avrdude -c arduino -p m328p -P COM3 -b 19200 -U lfuse:w:0xe2:m



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

// TODO Decide to use interrupt 
// With IRQ
// RAM:   [===       ]  32.8% (used 42 bytes from 128 bytes)
// Flash: [========= ]  91.1% (used 1866 bytes from 2048 bytes)
//
// diff: 68 bytes
//
// Without IRQ
// RAM:   [===       ]  32.8% (used 42 bytes from 128 bytes)
// Flash: [========= ]  87.8% (used 1798 bytes from 2048 bytes)
//
// diff: 30 bytes 
//
// With IRQ which only change state
// RAM:   [===       ]  32.8% (used 42 bytes from 128 bytes)
// Flash: [========= ]  89.3% (used 1828 bytes from 2048 bytes)
ISR(INT1_vect)
{
    state = State::ProcessIrq;
}


void configureMeasurementSensor()
{
    const bme280::ControlMeasurements ctrl {
        .ctrlTemperature = bme280::ControlOversamplingTemperature::OversamplingX2,
        .ctrlPressure = bme280::ControlOversamplingPressure::OversamplingX16,
        .mode = bme280::ControlMode::Sleep
    };
    bme280::writeControlMeasurements(ctrl);
}

void doMeasurements()
{
    bme280::startForceMeasurement();
    
    uint8_t counter = 10;
    do {
        if (counter == 0) 
            break;
        counter--;
        _delay_ms(10);
    } while (bme280::isBusy());
}

void readBatteryVoltage(uint8_t* data)
{
    *batteryVoltageEnablePort |= (1<<batteryVoltageEnablePin); // Enable voltage measurement
    eDELAY_MS(10); // Wait for voltage to stabilize
    *batteryVoltagePort &= ~(1<<batteryVoltagePin);
    
    data[0] = transmitLowLevelSPI(&data[1], &data[1], 0, 1);
    *batteryVoltagePort |= (1<<batteryVoltagePin);
    eDELAY_MS(10); // Wait before good reading from ADC
    *batteryVoltagePort &= ~(1<<batteryVoltagePin);
    data[0] = transmitLowLevelSPI(&data[1], &data[1], 0, 1);

    *batteryVoltagePort |= (1<<batteryVoltagePin);
    *batteryVoltageEnablePort &= ~(1<<batteryVoltageEnablePin); // Disable voltage measurement
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

    if(packet.General.header.errorFlag == 1)
        return 1;

    packet.Info.header.direction = sensorPacket::PacketDirection::Response;
    packet.Info.header.errorFlag = 0;

    readBytesEEPORM(IdentifierAddressEEPROM, packet.General.identifierRaw, 4);

    switch (packet.General.header.type)
    {
    case sensorPacket::PacketType::SensorInfo:
        readBytesEEPORM(SoftwareAddressEEPROM, packet.Info.softwareVersionRaw, 4);
        readBytesEEPORM(HardwareAddressEEPROM, packet.Info.hardwareVersionRaw, 4);
        bme280::readId(&packet.Info.sensorType);

        packetSize = 15;
        break;
    case sensorPacket::PacketType::SensorCalibData:
        eDELAY_MS(10);
        bme280::readCalibrationData(packet.CalibData.calibData);

        packetSize = 32;
        break;
    default:
        packet.Info.header.errorFlag = 1;
        break;
    } 
    return 0;
}

int main()
{
    DDRB = 0x00;
    DDRB |= (1 << PB5) | (1 << PB7) | (1 << ledPin0) | (1 << ledPin1) | (1 << batteryVoltageEnablePin) | (1 << batteryVoltagePin);
    PORTB = static_cast<uint8_t>((1 << buttonRightPin) | (1 << PB5) | (1 << PB7) | (1 << batteryVoltagePin));
    // PORTB |= (1<<buttonLeftPin);

    DDRD |= (1 << NRF24_CSN_PIN) | (1 << NRF24_CE_PIN) | (1 << BME280_CSN_PIN);
    // Config interrupt
    
    MCUCR = (1<<ISC11); // Set falling edgne on INT1 (irqPin)

    // MCUCR |= (1<<SE); // Enable sleep
    // MCUCR |= (1<<SM1) | (1<<SM0); // Configure sleep mode to Power Down
    GIFR = 0xf4;
    GIMSK = (1<<INT1); // Enable INT1 interrupt

    //init timer interrupt
    // TIMSK = (1<<TOIE1);
    // TIFR |= (1<<TOV1);
    // TCCR1B = (1<<CS11);
    turnOnRedLed();
    readBytesEEPORM(RxAddressEEPROM, rxAddress, 5);

    _delay_ms(100);
    intSPI(9600);
    nrf24::initnRF24(rxAddress, rxAddress);
    nrf24::setToPRX();    
    configureMeasurementSensor();
    
    turnOffLeds();
    sei();

    state = State::Idle; // TODO In future auto measuring will be processing

    uint8_t dataBuf[6]{0};
    
    blinkLed(LedColor::Green, BlinkCount::Initialized);

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
            doMeasurements();
            bme280::readAllDataBmp280(dataBuf);
            packet.Data.header.direction = sensorPacket::PacketDirection::Report;
            packet.Data.header.errorFlag = 0;
            packet.Data.header.type = sensorPacket::PacketType::SensorData;
            
            //packet.Data.identifierValue  = getSensorIdentifier();
            
            readBytesEEPORM(IdentifierAddressEEPROM, packet.Data.identifierRaw, 4);

            packet.Data.pressureRaw[2] = dataBuf[0];
            packet.Data.pressureRaw[1] = dataBuf[1];
            packet.Data.pressureRaw[0] = dataBuf[2];
            
            packet.Data.temperatureRaw[2] = dataBuf[3];
            packet.Data.temperatureRaw[1] = dataBuf[4];
            packet.Data.temperatureRaw[0] = dataBuf[5];
            
            readBatteryVoltage(packet.Data.batteryVoltageRaw);

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

        if(isButtonPressed(&PINB, buttonRightPin) && state == State::Idle)
        {
            state = State::DoMeasurements;
        }

        // if(isButtonPressed(&PINB, buttonLeftPin) && state == State::Idle)
        // {
        //     uint8_t bme280id = 0;
        //     bme280::readId(&bme280id);
        //     turnOnRedLed();
        //     eDELAY_MS(100);
        //     // turnOffLeds();
        //     packet.Data.header.direction = sensorPacket::PacketDirection::Report;
        //     packet.Data.header.errorFlag = 0;
        //     packet.Data.header.type = sensorPacket::PacketType::SensorInfo;
        //     packet.Data.identifierValue = getSensorIdentifier();
        //     // packet.Info.hardwareVersionValue = getHardwareVersion();
        //     // packet.Info.softwareVersionValue = getSoftwareVersion();
        //
        //     packet.Info.sensorType = bme280id;
        //
        //     packetSize = 12;
        //     // sensorPacket::generateCrc(packet.raw, 12);
        //
        //     // nrf24::setToPTX();
        //     // nrf24::sendData(packet.raw, 12);
        //     // state = State::Idle;
        //     state = State::SendPacket;
        // }
    }
}

#endif