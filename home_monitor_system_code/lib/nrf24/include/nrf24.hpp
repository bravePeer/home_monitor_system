#pragma once
#include <stdint.h>
#include "nrf24_config.hpp"
#include "port_types_def.hpp"
// #include "spi.hpp"

extern void eDELAY_MS(uint32_t);
extern void eDELAY_US(uint32_t);
extern portType NRF24_CE_PORT;
extern pinType NRF24_CE_PIN;
// extern portType NRF24_CSN_PORT;
// extern pinType NRF24_CSN_PIN;

extern uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len);

namespace nrf24
{   
    /// @brief Initializes nRF24.
    /// @warning IRQ and CE pin have to be initialized before this function!
    [[gnu::always_inline]] inline void initnRF24(const uint8_t* rxAddress, const uint8_t* txAddress)
    {
        // IRQ_DDR &= ~(1<<irqPIN);
        // CE_DDR |= (1<<cePIN);

        //Send address
        uint8_t data[5];

        data[0] = SetRegister(Reg::EnAA::enaa_p0);
        // transmitSPI(data, data, WriteRegister(RegMap::EnAA), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::EnAA), 1);
        eDELAY_MS(1);

        data[0] = SetRegister(Reg::EnRxAddr::erx_p0);
        // transmitSPI(data, data, WriteRegister(RegMap::EnRxAddr), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::EnRxAddr), 1);
        eDELAY_MS(1);

        data[0] = SetRegister(Reg::SetupAW::aw0, Reg::SetupAW::aw1);
        // transmitSPI(data, data, WriteRegister(RegMap::SetupAw), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::SetupAw), 1);
        eDELAY_MS(1);

        data[0] = 49;
        // transmitSPI(data, data, WriteRegister(RegMap::RFChannel), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::RFChannel), 1);
        eDELAY_MS(1);

        data[0] = SetRegister(Reg::RfSetup::rf_dr_low);
        // data[0] = SetRegister(Reg::RfSetup::rf_pwr1, Reg::RfSetup::rf_pwr2, Reg::RfSetup::rf_dr_low);
        // transmitSPI(data, data, WriteRegister(RegMap::RFSetup), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::RFSetup), 1);
        eDELAY_MS(1);

        // transmitSPI(rxAddress, data, WriteRegister(RegMap::RXAddressP0), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(rxAddress, data, WriteRegister(RegMap::RXAddressP0), 5);
        eDELAY_MS(1);

        // transmitSPI(txAddress, data, WriteRegister(RegMap::TXAddress), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(txAddress, data, WriteRegister(RegMap::TXAddress), 5);
        eDELAY_MS(1);

        data[0] = 32;
        // transmitSPI(data, data, WriteRegister(RegMap::RxPwP0), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::RxPwP0), 1);
        eDELAY_MS(1);


        data[0] = 0x6f;
        // transmitSPI(data, data, WriteRegister(RegMap::SetupRetr), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::SetupRetr), 1);
        eDELAY_MS(1);

        // Config for dynamic payload
        // data[0] = 0b00000011;
        data[0] = SetRegister(Reg::Dynpd::dpl_p0);
        // data[0] = 0x3f;
        // transmitSPI(data, data, WriteRegister(RegMap::DynPd), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::DynPd), 1);
        eDELAY_MS(1);

        // data[0] = 0b00000111;
        data[0] = SetRegister(Reg::Feature::en_dpl);
        // data[0] = 0;
        // transmitSPI(data, data, WriteRegister(RegMap::Feuture), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::Feuture), 1);
        eDELAY_MS(1);
        
        // Power up and add crc
        data[0] = SetRegister(Reg::Config::crc0, Reg::Config::en_crc, Reg::Config::pwr_up, Reg::Config::prim_rx);
        // transmitSPI(data, data, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::Config), 1);
        eDELAY_MS(100);
    }

    [[gnu::always_inline]] inline void setRxAddress(const uint8_t* rxAddress)
    {
        // transmitSPI(rxAddress, nullptr, WriteRegister(RegMap::RXAddressP0), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(rxAddress, nullptr, WriteRegister(RegMap::RXAddressP0), 5);
    }

    [[gnu::always_inline]] inline void setTxAddress(const uint8_t* txAddress)
    {
        // transmitSPI(txAddress, nullptr, WriteRegister(RegMap::TXAddress), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(txAddress, nullptr, WriteRegister(RegMap::TXAddress), 5);
    }
    
    [[gnu::always_inline]] inline void sendData(const uint8_t* data, uint8_t len)
    {
        // transmitSPI(data, nullptr, static_cast<uint8_t>(Commands::WriteTxPayload), len, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, nullptr, static_cast<uint8_t>(Commands::WriteTxPayload), len);
    }

    inline void setToPTX()
    {
        *NRF24_CE_PORT &= (~(1<<NRF24_CE_PIN));
        eDELAY_MS(5);
        uint8_t data = 0;
        // transmitSPI(nullptr, &data, ReadRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(nullptr, &data, ReadRegister(RegMap::Config), 1);
        eDELAY_MS(5);
        data &= ~SetRegister(Reg::Config::prim_rx);
        // transmitSPI(&data, nullptr, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(&data, nullptr, WriteRegister(RegMap::Config), 1);
        eDELAY_MS(5);
        *NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
        eDELAY_MS(5);
    }

    inline void setToPRX()
    {
        *NRF24_CE_PORT &= (~(1<<NRF24_CE_PIN));
        eDELAY_MS(5);
        uint8_t data = 0;
        // transmitSPI(nullptr, &data, ReadRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(nullptr, &data, ReadRegister(RegMap::Config), 1);
        data |= SetRegister(Reg::Config::prim_rx);
        eDELAY_MS(5);
        // transmitSPI(&data, nullptr, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(&data, nullptr, WriteRegister(RegMap::Config), 1);
        eDELAY_MS(5);
        *NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
        eDELAY_MS(5);
    }

    [[gnu::always_inline]] inline uint8_t getStatusReg()
    {
        // return transmitSPI(nullptr, nullptr, Cmd(Commands::Nop), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
        return transmitSpiNrf24(nullptr, nullptr, Cmd(Commands::Nop), 0);
    }

    [[gnu::always_inline]] inline uint8_t getConfig()
    {
        uint8_t data = 0;
        // transmitSPI(&data, &data, ReadRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(&data, &data, ReadRegister(RegMap::Config), 1);
        return data;
    }
    
    [[gnu::always_inline]] inline void writeCommand(Commands command)
    {
        // transmitSPI(nullptr, nullptr, static_cast<uint8_t>(command), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(nullptr, nullptr, static_cast<uint8_t>(command), 0);
    }

    /// @brief 
    /// @param data 32 byte buffer
    /// @return read bytes count
    inline uint8_t readRxPayload(uint8_t* data)
    {
        uint8_t dataCount = 0;
        // transmitSPI(&dataCount, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(&dataCount, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1);
        if(dataCount > 32)
        {
            // transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
            transmitSpiNrf24(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0);
            return 0;
        }
        // transmitSPI(data, data, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCount, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCount);
        return dataCount;
    }

    [[gnu::always_inline]] inline uint8_t getRxPayloadCount()
    {
        uint8_t dataCount = 0;
        // transmitSPI(&dataCount, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(&dataCount, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1);
        return dataCount;
    }

    [[gnu::always_inline]] inline void readRxPayloadRaw(uint8_t* data, uint8_t dataCount)
    {
        // transmitSPI(data, data, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCount, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCount);
    }

    [[gnu::always_inline]] inline void flushTx()
    {
        // transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0);
    }

    [[gnu::always_inline]] inline void flushRx()
    {
        // transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0);
    }
}
