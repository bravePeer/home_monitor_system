#ifdef ATTINY2313A
extern "C"
{
  #include "avr/io.h"
  #include "util/delay.h"
  #include "avr/interrupt.h"
  #include "avr/sleep.h"
}
#include "spi.h"

#define NRF24_CSN_PORT PORTD
#define NRF24_CSN_PIN (uint8_t)PD4
#define NRF24_CE_PORT PORTD
#define NRF24_CE_PIN (uint8_t)PD5
#define NRF24_IRQ_PORT PORTD
#define NRF24_IRQ_PIN (uint8_t)PD3
#define DELAY_MS(val) _delay_ms(val)
#define DELAY_US(val) _delay_us(val)
#include "nRF24.h"
#define BME280_CSN_PORT PORTD
#define BME280_CSN_PIN (uint8_t)PD6
#include "bme280.h"
#include "sensor/sensor_packet.hpp"
// Attiny2313 -> gdy uśpione 18 uA
// REad fueses
// avrdude.exe  -p ATtiny2313A -c usbasp -U hfuse:r:-:h -U lfuse:r:-:h

// Write fuses
// avrdude -c arduino -p m328p -P COM3 -b 19200 -U lfuse:w:0xe2:m

// Attiny                     Rp
//  Button  
//  pressed ------Send----->
//         <-----Send------ response
// if no response retransmit


#ifdef ATTINY2313A
constexpr uint8_t buttonRightPin = PB0;
constexpr uint8_t buttonLeftPin = PB1;
constexpr uint8_t ledPIN0 = PB2;
constexpr uint8_t ledPIN1 = PB3;
#define LED_DDR DDRB
#define LED_PORT PORTB

uint8_t dataToProcess[32];
uint8_t dataCoutToProcess;
uint8_t shouldSend = 0;

volatile uint8_t shouldBlink = 0;

enum  State
{
  Sleep,
  SleepPwrDown,
  SleepStandby,
  StartSending,
  Sending,
  Receiving,
  Idle
};

volatile State state = State::Sleep;

void turnOffLeds()
{
  LED_PORT &= ~((1<<ledPIN0) | (1<<ledPIN1));
}

void turnOnRedLed()
{
  turnOffLeds();
  LED_PORT |= (1<< ledPIN0);
}

void turnOnGreenLed()
{
  turnOffLeds();
  LED_PORT |= (1<< ledPIN1);
}

void setDone()
{
  PORTD |= (1<<PD6);
  _delay_ms(1);
  PORTD &= ~(1<<PD6);
  _delay_ms(1);
}

volatile uint8_t prescaler = 0;

ISR(PCINT0_vect)
{
  return;
  _delay_ms(1);
  if(~PINB & (1<<buttonLeftPin))
  {
    state = State::SleepPwrDown;
    return;

    if(prescaler)
    {
      prescaler = 0;
      cli();
      CLKPR = (1<<CLKPCE);
      CLKPR = 0;
      sei();
      turnOnGreenLed();
    }
    else
    {
      prescaler = 1;
      cli();
      CLKPR = (1<<CLKPCE);
      CLKPR = (1<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0);
      sei();
      turnOnRedLed();
    }
    // if(state != State::Sleep)
    //   return;
    // turnOnRedLed();
    // _delay_ms(1000);

    // state = State::Sleep;
  }
  else if(~PINB & (1<<buttonRightPin))
  {
    state = State::SleepStandby;
    return;
    // if(state != State::Sleep)
    //   return;
    // TCCR1B = 0;
    state = State::StartSending;

    // TODO disable interrupts on pcint
  }
}

ISR(INT1_vect)
{
  state = State::Idle;

  NRF24_CE_PORT &= ~(1<<NRF24_CE_PIN);
  // _delay_us(1);
  uint8_t status = transmitSPI(nullptr, nullptr, WriteCmd(Commands::Nop), 0, &NRF24_CSN_PORT, NRF24_CSN_PIN);
  
  
  switch (status & SetRegister(Reg::Status::max_rt, Reg::Status::tx_ds, Reg::Status::rx_dr))
  {
    case SetRegister(Reg::Status::rx_dr): // Data ready, received data
      transmitSPI(&dataCoutToProcess, &dataCoutToProcess, WriteCmd(Commands::ReadRxPayloadWidth), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
      // dataCoutToProcess = dataToProcess[0];
      
      transmitSPI(dataToProcess, dataToProcess, WriteCmd(Commands::ReadRxPayload), dataCoutToProcess, &NRF24_CSN_PORT, NRF24_CSN_PIN);
      shouldSend = 1;
      turnOnGreenLed();

      break;
    case SetRegister(Reg::Status::max_rt):
      transmitSPI(nullptr, nullptr, WriteCmd(Commands::FlushTx), 0, &NRF24_CSN_PORT, NRF24_CSN_PIN);
      turnOnRedLed();
      break;
    case SetRegister(Reg::Status::tx_ds):
      turnOnGreenLed();
    //   nrf24::setToPRX();
      break;
  }

  uint8_t resetReg = 0x70;
  transmitSPI(&resetReg, &resetReg, WriteRegister(RegMap::Status), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
  NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
}

volatile uint8_t timerCounter = 0;
ISR(TIMER1_OVF_vect)
{
  cli();
  if((timerCounter & 0x01) == 0)
    turnOffLeds();
  else
    turnOnGreenLed();
  timerCounter++;
  sei();
}

__attribute__((noreturn)) void testMainLoop()
{
  uint8_t temperatureSensor = 0;
  nrf24::setToPRX();

  while(true)
  {
    if(!(PINB & (1<<buttonLeftPin)))
    {
      _delay_ms(10);
      if(!(PINB & (1<<buttonLeftPin)))
      {
        turnOnRedLed();
        _delay_ms(1000);
        turnOffLeds();
        // nrf24::nRF24Packet packet;
        // packet.header = nrf24::nRF24Packet::Header::TestDevice;
        // packet.temperature[0] = temperatureSensor;
        // for (uint8_t i = 2; i < nrf24::nRF24PacketSize; i++)
        //   packet.raw[i] = 'a' + i;
        // nrf24::sendData(packet.raw, nrf24::nRF24PacketSize);

        // Theoretically this is more memory efficient
        sensorPacket::SensorPacket packet
        {
          .Info = {
            .header = {
              .direction = sensorPacket::PacketDirection::Response,
              .errorFlag = 0,
              .type = sensorPacket::PacketType::SensorInfo
            },
            .crc = 2,
            {.identifierValue = 0},
            {.sensorType = 1},
            {.softwareVersionValue = 1},
            {.hardwareVersionValue = 1}
          }
        };
        // Theoretically this is less memory efficient
        // sensorPacket::SensorPacket packet;
        // packet.Info.header = {
        //     .direction = sensorPacket::PacketDirection::Response,
        //     .errorFlag = 0,
        //     .type = sensorPacket::PacketType::SensorInfo
        // };
        // packet.Info.crc = 2;
        // // packet.Info.identifierValue = 0;
        // packet.Info.identifierRaw[0] = 0;
        // packet.Info.sensorType = 1;
        // // packet.Info.hardwareVersionValue = 1;
        // // packet.Info.softwareVersionValue = 1;
        // packet.Info.hardwareVersionRaw[0] = 1;
        // packet.Info.softwareVersionRaw[0] = 1;

            nrf24::setToPTX();
            nrf24::sendData(packet.raw, 15);
      }
    }

    if(!(PINB & (1<<buttonRightPin)))
    {
      _delay_ms(10);
      if(!(PINB & (1<<buttonRightPin)))
      {
        turnOnRedLed();
        _delay_ms(1000);
        turnOffLeds();

        // sensorPacket::SensorPacket packet
        // {
        //     .Data = {
        //         .header = {
        //             .direction = sensorPacket::PacketDirection::Response,
        //             .errorFlag = 0,
        //             .type = sensorPacket::PacketType::SensorData
        //         },
        //         .crc = 2,
        //         {.identifierValue = 0},
        //         {.temperatureValue = temperatureSensor},
        //         {.pressureValue = 1},
        //         {.humidityValue = 1},
        //         {.batteryVoltageValue = 1}
        //     }
        // };

        sensorPacket::SensorPacket packet;
        packet.Data.header = {
                    .direction = sensorPacket::PacketDirection::Response,
                    .errorFlag = 0,
                    .type = sensorPacket::PacketType::SensorData
                };
        packet.Data.crc = 2;
        packet.Data.identifierValue = 0;

        nrf24::setToPTX();
        nrf24::sendData(packet.raw, 22);
      }
    }
    temperatureSensor += 1;
  }
}

int main()
{
  // Testing TPL
  // DDRD |= (1<<PD6);
//   PORTD = 0;
  //
//   DDRB = 0x00;
//   LED_DDR |= (1 << ledPIN0) | (1 << ledPIN1);
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
  LED_DDR |= (1 << ledPIN0) | (1 << ledPIN1);
  PORTB = (1<<buttonLeftPin) | (1<<buttonRightPin);

  DDRD |= (1<<NRF24_CSN_PIN) | (1<<NRF24_CE_PIN);
  NRF24_CSN_PORT |= (1<<NRF24_CSN_PIN);

  // Config interrupt
  MCUCR = (1<<ISC11); // Set falling edgne on INT1 (irqPin)
  // MCUCR |= (1<<SE); // Enable sleep
  // MCUCR |= (1<<SM1) | (1<<SM0); // Configure sleep mode to Power Down
  GIFR = 0xf4;
  GIMSK = (1<<INT1) | (1<<PCIE0); 
  PCMSK0 = (1<<PCINT0) | (1<<PCINT1);
  uint8_t data = 1;
  // uint8_t receivedBuf[5];
  // uint8_t sendBuf[5] = {0x12, 0x23, 0x61, 0xdd, 0xab};
  // NRF24_CE_PORT |= (1<<NRF24_CE_PIN); // Active
  // set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  //init timer interrupt
  // TIMSK = (1<<TOIE1);
  // TIFR |= (1<<TOV1);
  // TCCR1B = (1<<CS11);

  // turnOnGreenLed();


  turnOnRedLed();
  intSPI(9600);
  nrf24::initnRF24(); // nrf is in PRX
  turnOffLeds();

  sei();
  // Power down
  // dd = 0;
  // transmitSPI(&dd, &dd, ReadRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
  // dd &= ~SetRegister(Reg::Config::pwr_up);
  // transmitSPI(&dd, &dd, WriteRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);

  

  testMainLoop();
  
  
  state = State::Sending;
  


  while (true)
  {
    switch (state)
    {
    case State::StartSending:
    {
      turnOnGreenLed();
      _delay_ms(1000);

      state = State::Sending;
      nrf24::setToPTX();

      uint8_t sendBuf2[5] = {data, 0x23, 0x61, 0xdd, 0xab};
      nrf24::sendData(sendBuf2, 5);

      data++;
      nrf24::setToPRX();
      NRF24_CE_PORT |= (1<<NRF24_CE_PIN); // Receiving mode
    
      break;    
    }
    case State::Receiving:
    break;
    case State::Sending:
    break;
    case State::Idle:

    break;
    case State::Sleep:
    {
      cli();

      // state = State::Idle;
      LED_PORT &= ~((1<<ledPIN0) | (1<<ledPIN1));
      LED_PORT |= (1<< ledPIN1);
      // _delay_ms(100);
      LED_PORT &= ~((1<<ledPIN0) | (1<<ledPIN1));

      BODCR = 3;
      BODCR = 2;
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      sei();
      break;
    }
    case State::SleepPwrDown:
      turnOnRedLed();
      _delay_ms(100);
      turnOffLeds();
      MCUCR |= (1<<SM1) | (1<<SM0); // Configure sleep mode to Power Down
      // set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      sei();
      break;
    case State::SleepStandby:
      turnOnGreenLed();
      _delay_ms(100);
      turnOffLeds();
      MCUCR &= ~(1<<SM0);
      MCUCR |=  (1<<SM1); // Configure sleep mode to Standby
      // set_sleep_mode(SLEEP_MODE_STANDBY);
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      sei();
      break;
    }


    // if(!(PINB & (1<<buttonLeftPin)))
    // {
    //   CE_PORT &= ~(1<<cePIN); // Clean receiving mode
    //   setToPTX();

    //   LED_PORT &= ~(1<<ledPIN0);
    //   LED_PORT |= (1<<ledPIN1);

    //   // transmitSPI(sendBuf, receivedBuf, static_cast<uint8_t> (Commands::ReadRegister) | static_cast<uint8_t>(RegisterAddress::Config), 5);
    //   // transmitSPI(sendBuf, receivedBuf, static_cast<uint8_t>(Commands::WriteTXPayload), 5);
    //   uint8_t sendBuf2[5] = {data, 0x23, 0x61, 0xdd, 0xab};
    //   sendData(sendBuf2, 5);

    //   data++;
    //   setToPRX();
    //   CE_PORT |= (1<<cePIN); // Receiving mode
    
    //   _delay_ms(500);
    // }

    // if(!(PINB & (1<<buttonRightPin)))
    // {
    //   LED_PORT &= ~(1<<ledPIN1);
    //   LED_PORT |= (1<<ledPIN0);

    //   transmitSPI(sendBuf, receivedBuf, ReadRegister(RegMap::Config), 1);
    //   _delay_ms(10);

    //   sendBuf[0] = 0x70;
    //   transmitSPI(sendBuf, receivedBuf, WriteRegister(RegMap::Status), 1);

    //   data++;
    //   _delay_ms(100);
    // }
  }

  return 0;
}
#elif ATMEGA328P
#include "usart.h"

//DDRD
constexpr uint8_t ledPIN = PD2;
//DDRD
constexpr uint8_t buttonLeftPin = PD4;

// void turnOffLeds()
// {
//   LED_PORT &= ~((1<<ledPIN0) | (1<<ledPIN1));
// }

// void turnOnRedLed()
// {
//   turnOffLeds();
//   LED_PORT |= (1<< ledPIN0);
// }

// void turnOnGreenLed()
// {
//   turnOffLeds();
//   LED_PORT |= (1<< ledPIN1);
// }


ISR(INT1_vect)
{
  uint8_t status = transmitSPI(nullptr, nullptr, WriteCmd(Commands::Nop), 0);
  uint8_t dataToProcess[32];
  uint8_t dataCoutToProcess;
  
  switch (status & SetRegister(Reg::Status::max_rt, Reg::Status::tx_ds, Reg::Status::rx_dr))
  {
    case SetRegister(Reg::Status::rx_dr): // Data ready, received data
      transmitSPI(dataToProcess, dataToProcess, WriteCmd(Commands::ReadRxPayloadWidth), 1);
      dataCoutToProcess = dataToProcess[0];
      
      transmitSPI(dataToProcess, dataToProcess, WriteCmd(Commands::ReadRxPayload), dataCoutToProcess);

      for (uint8_t i = 0; i < dataCoutToProcess; i++)
      {
        transmitUART(dataToProcess[i]);
      }
      

      break;
    case SetRegister(Reg::Status::max_rt):
      break;
    case SetRegister(Reg::Status::tx_ds):
      break;
  }
  _delay_us(100);
  uint8_t resetReg = 0x70;
  transmitSPI(&resetReg, &resetReg, WriteRegister(RegMap::Status), 1);
  // transmitSPI(nullptr, nullptr, WriteCmd(Commands::FlushRx), 1);
  
  // CE_PORT &= ~(1<<cePIN); // Receiving mode active
  // _delay_us(100);
  // CE_PORT |= (1<<cePIN); // Receiving mode active
  
}

int main()
{
  initUSART(usartUBRR);
  DDRD |= (1<<ledPIN);
  PORTD |= (1<<ledPIN) | (1<<buttonLeftPin);
  initSPI();
  initnRF24();

  uint8_t data = 1;
  uint8_t receivedBuf[5];
  uint8_t sendBuf[5] = {0xaa, 0xaa, 0xaa, 0xaa, 0xaa};
  transmitSPI(nullptr, nullptr, WriteCmd(Commands::FlushRx), 0);
  setToPRX();
  MCUCR = (1<<ISC11);
  EIMSK = (1<<INT1);
  
  sei();

  char info[] = "ready\n";
  for (uint8_t i = 0; i < 6; i++)
  {
    transmitUART(info[i]);
  }
  
  CE_PORT |= (1<<cePIN); // Receiving mode active

  while (1)
  {
    //Send some data
    // if(!(PIND & (1<<buttonLeftPin)))
    // {
    //   CE_PORT &= ~(1<<cePIN);
    //   setToPTX();
    //   PORTD = PORTD & ~(1<<ledPIN) | (((~(PORTD & (1<<ledPIN))) & (1<<ledPIN)));
      
    //   for (uint8_t i = 0; i < 5; i++)
    //   {
    //     transmitUART(sendBuf[i]);
    //   }
    //   uint8_t copyToSend[5];
    //   for (uint8_t i = 0; i < 5; i++)
    //   {
    //     copyToSend[i] = sendBuf[i];
    //   }
      
    //   sendData(copyToSend, 5);
      
    //   setToPRX();

    //   CE_PORT |= (1<<cePIN); // Receiving mode

    //   sendBuf[0]++;

    //   // uint8_t status = transmitSPI(sendBuf, receivedBuf, ReadRegister(RegMap::Config), 1);
    //   // transmitUART(status);
      
      
      
    //   // transmitSPI(sendBuf, receivedBuf, WriteCmd(Commands::ReadRxPayloadWidth),1);
    //   // uint8_t width = receivedBuf[0];
    //   // transmitUART(width);
      
    //   // transmitSPI(sendBuf, receivedBuf, WriteCmd(Commands::ReadRxPayload), width);
    //   // for (uint8_t i = 0; i < width; i++)
    //   // {
    //   //   transmitUART(receivedBuf[i]);
    //   // }
      
    //   // uint8_t resetReg[1];
    //   // resetReg[0] = 0x70;
    //   // transmitSPI(resetReg, resetReg, ReadRegister(RegMap::Status), 1); // Should be write?

    //   // transmitSPI(nullptr, nullptr, WriteCmd(Commands::FlushRx), 0);
    //   // _delay_ms(1);
    //   _delay_ms(500);
    //   // CE_PORT |= (1<<cePIN);
    // }
  }

  return 0;
}
#endif
#endif