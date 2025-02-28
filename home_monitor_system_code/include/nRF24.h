#pragma once
#include <stdint.h>
#include "spi.h"
#include "nRF24Config.h"

// #ifdef ATTINY2313A
// // #define IRQ_DDR DDRD
// // #define CE_DDR DDRD
// // #define IRQ_PORT PORTD
// // #define CE_PORT PORTD
// // constexpr uint8_t irqPIN = PD3; // INT1
// // constexpr uint8_t cePIN = PD5;
//
// #elif ATMEGA328P
// #define IRQ_DDR DDRD
// #define CE_DDR DDRB
// #define IRQ_PORT PORTD
// #define CE_PORT PORTB
// //DDRB
// constexpr uint8_t irqPIN = PD3;
// //DDRB
// constexpr uint8_t cePIN = PB1;
// // constexpr static volatile uint8_t* const cePort()  {return &DDRB; };
// #endif

uint8_t nRF24Address[5] = {'a','a','a','a','a'};


#if !(defined(NRF24_CSN_PORT) && defined(NRF24_CSN_PIN))
#error Define NRF24_CSN_PORT and NRF24_CSN_PIN!
#endif

#if !(defined(NRF24_CE_PORT) && defined(NRF24_CE_PIN))
#error Define NRF24_CE_PORT and NRF24_CE_PIN!
#endif

#if !(defined(NRF24_IRQ_PORT) && defined(NRF24_IRQ_PIN))
#error Define NRF24_IRQ_PORT and NRF24_IRQ_PIN!
#endif

#if !(defined(DELAY_MS) && defined(DELAY_US))
#error Define DELAY_MS() and DELAY_US()
#endif

namespace nrf24
{
  /// @brief Initializes nRF24.
  /// @warning IRQ and CE pin have to be initialized before this function!
  void initnRF24()
  {
    DELAY_MS(100);

    // IRQ_DDR &= ~(1<<irqPIN);
    // CE_DDR |= (1<<cePIN);

    //Send address
    uint8_t data[5];

    data[0] = SetRegister(Reg::EnAA::enaa_p0);
    transmitSPI(data, data, WriteRegister(RegMap::EnAA), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = SetRegister(Reg::EnRxAddr::erx_p0);
    transmitSPI(data, data, WriteRegister(RegMap::EnRxAddr), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = SetRegister(Reg::SetupAW::aw0, Reg::SetupAW::aw1);
    transmitSPI(data, data, WriteRegister(RegMap::SetupAw), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = 76;
    transmitSPI(data, data, WriteRegister(RegMap::RFChannel), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = SetRegister(Reg::RfSetup::rf_dr_low);
    // data[0] = SetRegister(Reg::RfSetup::rf_pwr1, Reg::RfSetup::rf_pwr2, Reg::RfSetup::rf_dr_low);
    transmitSPI(data, data, WriteRegister(RegMap::RFSetup), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    transmitSPI(nRF24Address, data, WriteRegister(RegMap::RXAddressP0), 5, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    transmitSPI(nRF24Address, data, WriteRegister(RegMap::TXAddress), 5, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = 5;
    transmitSPI(data, data, WriteRegister(RegMap::RxPwP0), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = 0x2f;
    transmitSPI(data, data, WriteRegister(RegMap::SetupRetr), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);


    data[0] = 0b00000011;
    transmitSPI(data, data, WriteRegister(RegMap::DynPd), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    data[0] = 0b00000111;
    transmitSPI(data, data, WriteRegister(RegMap::Feuture), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(1);

    /*
  

    // #ifdef ATMEGA328P
    // data[0] = 0b00001111;
    // #else
    // data[0] = 0b00001110;
    // #endif
    */
    
    data[0] = SetRegister(Reg::Config::crc0, Reg::Config::en_crc, Reg::Config::pwr_up, Reg::Config::prim_rx);
    transmitSPI(data, data, WriteRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_MS(100);
  }

  /*void sendData(uint8_t* data, uint8_t len)
  {
    // transmitSPI(nullptr,nullptr, static_cast<uint8_t>(Commands::FlushTx),0);
    DELAY_US(50);
    // Send to TX buffer
    transmitSPI(data, data, static_cast<uint8_t>(Commands::WriteTxPayload), len, &NRF24_CSN_PORT, NRF24_CSN_PIN);

    // Set PRIM_RX = 0

    // Set CE = 1
    DELAY_MS(10);
    CE_PORT |= (1<<cePIN);
    DELAY_US(20);
    CE_PORT &= ~(1<<cePIN);
    DELAY_MS(10);  
  }*/

 void sendData(uint8_t* data, uint8_t len)
  {
    // transmitSPI(nullptr,nullptr, static_cast<uint8_t>(Commands::FlushTx),0);
    DELAY_US(50);
    // Send to TX buffer
    transmitSPI(data, data, static_cast<uint8_t>(Commands::WriteTxPayload), len, &NRF24_CSN_PORT, NRF24_CSN_PIN);

    // Set PRIM_RX = 0

    // Set CE = 1
    DELAY_MS(10);
    NRF24_CE_PORT = NRF24_CE_PORT | ((1<<NRF24_CE_PIN));
    // DELAY_US(20); //modified 23.01.2025
    // NRF24_CE_PORT = NRF24_CE_PORT & (~(1<<NRF24_CE_PIN)); //modified 23.01.2025
    // DELAY_MS(10);  //modified 23.01.2025
  }

  inline void setToPTX()
  {
    NRF24_CE_PORT = NRF24_CE_PORT & (~(1<<NRF24_CE_PIN));
    uint8_t data = 0;
    transmitSPI(&data, &data, ReadRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_US(100);
    data &= ~SetRegister(Reg::Config::prim_rx);
    transmitSPI(&data, &data, WriteRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    // NRF24_CE_PORT = NRF24_CE_PORT | ((1<<NRF24_CE_PIN));
  }

  inline void setToPRX()
  {
    uint8_t data = 0;
    transmitSPI(&data, &data, ReadRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    data |= SetRegister(Reg::Config::prim_rx);
    DELAY_US(100);
    transmitSPI(&data, &data, WriteRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    DELAY_US(500);
    NRF24_CE_PORT = NRF24_CE_PORT | ((1<<NRF24_CE_PIN));
  }


  // ---Start of new code---
  inline uint8_t getStatusReg()
  {
    return transmitSPI(nullptr, nullptr, Cmd(Commands::Nop), 0, &NRF24_CSN_PORT, NRF24_CSN_PIN);
  }

  inline uint8_t getConfig()
  {
    uint8_t data = 0;
    transmitSPI(&data, &data, ReadRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    return data;
  }

  // ---End of new code---

  /*
  inline void setToPTX()
  {
    CE_PORT &= ~(1<<cePIN);
    uint8_t data = 0;
    transmitSPI(&data, &data, ReadRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    data &= ~SetRegister(Reg::Config::prim_rx);
    transmitSPI(&data, &data, WriteRegister(RegMap::Config), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
    CE_PORT |= (1<<cePIN);
  }
  */
}

// void sendData(volatile uint8_t* data, uint8_t len)
// {
//   // transmitSPI(nullptr,nullptr, static_cast<uint8_t>(Commands::FlushTx),0);
//   DELAY_US(50);
//   // Send to TX buffer
//   transmitSPI(data, data, static_cast<uint8_t>(Commands::WriteTxPayload), len);
//   // Set PRIM_RX = 0
//   // Set CE = 1
//   DELAY_MS(10);
//   CE_PORT |= (1<<cePIN);
//   DELAY_US(20);
//   CE_PORT &= ~(1<<cePIN);
//   DELAY_MS(10);  
// }