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

// Attiny2313 -> sleep 18 uA
// Read fueses
// avrdude.exe  -p ATtiny2313A -c usbasp -U hfuse:r:-:h -U lfuse:r:-:h

// Write fuses
// avrdude -c arduino -p m328p -P COM3 -b 19200 -U lfuse:w:0xe2:m

portType BME280_CSN_PORT = &PORTD;
pinType BME280_CSN_PIN = PD6;

portType NRF24_CSN_PORT = &PORTD;
pinType NRF24_CSN_PIN = PD5;
portType NRF24_CE_PORT = &PORTD;
pinType NRF24_CE_PIN = PD4;
portType NRF24_IRQ_PORT = &PORTD;
pinType NRF24_IRQ_PIN = PD3;

portType batteryVoltageEnablePort = &PORTB;
pinType batteryVoltageEnablePin = PB1;
portType batteryVoltagePort = &PORTB;
pinType batteryVoltagePin = PB2;

portType ledDir = &DDRB;
portType ledPort = &PORTB;
pinType ledPin0 = PB4;
pinType ledPin1 = PB3;

constexpr uint8_t buttonRightPin = PB0;
// constexpr uint8_t buttonLeftPin = PB1;


void eDELAY_MS(uint32_t val)
{
    _delay_ms(static_cast<double>(val));
}

void eDELAY_US(uint32_t val)
{
    _delay_us(static_cast<double>(val));
}

uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *NRF24_CSN_PORT &= (~(1<<NRF24_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *NRF24_CSN_PORT |= (1<<NRF24_CSN_PIN);
    return ret;
    // return transmitSPI(sendBuf, receiveBuf, cmd, len, NRF24_CSN_PORT, NRF24_CSN_PIN);
}

uint8_t transmitSpiBme280(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *BME280_CSN_PORT &= (~(1<<BME280_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *BME280_CSN_PORT |= (1<<BME280_CSN_PIN);
    return ret;
    // return transmitSPI(sendBuf, receiveBuf, cmd, len, NRF24_CSN_PORT, NRF24_CSN_PIN);
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

constexpr uint32_t getSensorIdentifier()
{
    return 0;
}

constexpr uint32_t getHardwareVersion()
{
    return 0;
}

constexpr uint32_t getSoftwareVersion()
{
    return 0;
}

enum class State : uint8_t
{
    ToIdle,
    ToIdleMaxRetr,
    Idle,
    PwrUp,
    PwrDown,
    ProcessReceivedData,
    DoMeasurements,
    SendPacket,
    ProcessIrq
};

volatile State state;

void turnOffLeds()
{
    *ledPort &= ~((1<<ledPin0) | (1<<ledPin1));
}

void turnOnRedLed()
{
    turnOffLeds();
    *ledPort |= (1<<ledPin0);
}

void turnOnGreenLed()
{
    turnOffLeds();
    *ledPort |= (1<<ledPin1);
}

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
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::max_rt))
    {
        turnOnRedLed();
        // eDELAY_MS(100);
        state = State::ToIdleMaxRetr;
    }

    if(status & nrf24::SetRegister(nrf24::Reg::Status::tx_ds))
    {
        turnOnGreenLed();
        // transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
        // eDELAY_MS(100);
        state = State::ToIdle;
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
    state = State::ProcessIrq; // Timming errors?
    // uint8_t status = nrf24::getStatusReg();
    // if(status & nrf24::SetRegister(nrf24::Reg::Status::rx_dr))
    // {
    //     state = State::ProcessReceivedData;
    // }

    // if(status & nrf24::SetRegister(nrf24::Reg::Status::max_rt))
    // {
    //     turnOnRedLed();
    //     // eDELAY_MS(100);
    //     state = State::ToIdleMaxRetr;
    // }

    // if(status & nrf24::SetRegister(nrf24::Reg::Status::tx_ds))
    // {
    //     turnOnGreenLed();
    //     // transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     // eDELAY_MS(100);
    //     state = State::ToIdle;
    // }
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
    
    uint8_t counter = 3;
    do {
        if (counter == 0) 
            break;
        counter--;
        _delay_ms(10);
    } while (bme280::isBusy());
}

int8_t isButtonPressed(portType port, uint8_t pin)
{
    if(!(*port & (1<<pin)))
    {
        _delay_ms(10);
        if(!(*port & (1<<pin)))
        {
            return 1;
        }
    }
    return 0;
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
        return 1;
    }
    turnOnGreenLed();
    //eDELAY_MS(100);
    if(packet.General.header.direction != sensorPacket::PacketDirection::Request)
        return 1;

    if(packet.General.header.errorFlag == 1)
        return 1;

    packet.Info.header.direction = sensorPacket::PacketDirection::Response;
    packet.Info.header.errorFlag = 0;
    // packet.Info.header.type = sensorPacket::PacketType::SensorInfo;
    packet.CalibData.identifierValue = getSensorIdentifier();

    switch (packet.General.header.type)
    {
    case sensorPacket::PacketType::SensorInfo:
        // packet.Info.header.direction = sensorPacket::PacketDirection::Response;
        // packet.Info.header.errorFlag = 0;
        // packet.Info.header.type = sensorPacket::PacketType::SensorInfo;
        // packet.Info.identifierValue = getSensorIdentifier();
        packet.Info.hardwareVersionValue = 0;
        packet.Info.softwareVersionValue = 0;
        bme280::readId(&packet.Info.sensorType);

        packetSize = 16;
        break;
    case sensorPacket::PacketType::SensorCalibData:
        // packet.CalibData.header.direction = sensorPacket::PacketDirection::Response;
        // packet.CalibData.header.errorFlag = 0;
        // packet.CalibData.header.type = sensorPacket::PacketType::SensorCalibData;
        // packet.CalibData.identifierValue = getSensorIdentifier();
        eDELAY_MS(10);
        bme280::readCalibrationData(packet.CalibData.calibData);
        packetSize = 32;
        
        break;
    default:
        packet.CalibData.header.direction = sensorPacket::PacketDirection::Response;
        packet.Info.header.errorFlag = 1;
        break;
    } 
    return 0;
}

const uint8_t rxAddress[5] = {'a', 'b', 'c', 'd', '0'};
// const uint8_t txAddress[5] = {'a', 'b', 'c', 'd', '0'};

int main()
{
  // Testing TPL
  // DDRD |= (1<<PD6);
//   PORTD = 0;
  //
//   DDRB = 0x00;
//   LED_DDR |= (1 << ledPin0) | (1 << ledPin1);
//   PORTB = (1<<buttonLeftPin) | (1<<buttonRightPin);
//
//   turnOnGreenLed();
//   _delay_ms(500);
//   // turnOnRedLed();
//   turnOffLeds();
//  _delay_ms(500);
  //
//   intSPI(1000u);
//   DDRD |= (1<<csnPIN);
//   DDRD |= (1<<PD6);
  //
//   PORTD |= (1<<PD6);
//
//   DDRD &= ~(1<<NRF24_IRQ_PIN);
//   DDRD |= (1<<NRF24_CE_PIN);
//   nrf24::initnRF24(); // nrf is in PRX
//
//   uint8_t dd = 0;
  //
//   // transmitSPI(&dd, &dd, ReadRegister(RegMap::Config), 1);
//   // dd &= ~SetRegister(Reg::Config::pwr_up);
//   // transmitSPI(&dd, &dd, WriteRegister(RegMap::Config), 1);
//   turnOnGreenLed();
//   _delay_ms(1000);
//   turnOffLeds();
//   nrf24::setToPTX();
//   uint8_t sendBuf2[5] = {'a', 'b', 'c', 'd', 'e'};
//   nrf24::sendData(sendBuf2, 5);
//   NRF24_CE_PORT |= (1<<NRF24_CE_PIN); 
//   _delay_ms(5000);
//   while (1)
//   {
//     if(!(PINB & (1<<buttonLeftPin)))
//     {
//       _delay_ms(10);
//       if(!(PINB & (1<<buttonLeftPin)))
//       {
//         turnOffLeds();
//         // Send
//         // state = State::Sending;
//         nrf24::setToPTX();
//         uint8_t sendBuf3[5] = {'a', 'b', 'c', 'd', 'e'};
//         nrf24::sendData(sendBuf3, 5);
//         NRF24_CE_PORT |= (1<<NRF24_CE_PIN); 
//         _delay_ms(500);
//         NRF24_CE_PORT &= ~(1<<NRF24_CE_PIN); 
//         nrf24::setToPRX();
//         NRF24_CE_PORT |= (1<<NRF24_CE_PIN); // Receiving mode
//         turnOnGreenLed();
//         _delay_ms(1000);
//         turnOffLeds();
//         setDone();
//       }
//     }
//   }
  
  
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
    _delay_ms(100);
    turnOnRedLed();
    intSPI(9600);
    nrf24::initnRF24(rxAddress, rxAddress);
    turnOffLeds();
    nrf24::setToPRX();

    sei();

    configureMeasurementSensor();

    state = State::Idle; // TODO In future auto measuring will be processing

    uint8_t dataBuf[6]{0};
    
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
            // if(!(PIND & (1<<NRF24_IRQ_PIN)))
            // {
            //     state = State::ProcessIrq;
            // }
            break;
        case State::DoMeasurements:
            doMeasurements();
            bme280::readAllDataBmp280(dataBuf);
            packet.Data.header.direction = sensorPacket::PacketDirection::Report;
            packet.Data.header.errorFlag = 0;
            packet.Data.header.type = sensorPacket::PacketType::SensorData;
            
            packet.Data.identifierValue = getSensorIdentifier();
            
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