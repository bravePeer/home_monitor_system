#pragma once
#include <stdint.h>
#if __has_include(<utility>)
    #include <utility>
#else
    #include <traits.hpp>
    namespace std
    {
        template<typename T>
        constexpr auto to_underlying(T val) noexcept 
        {
            return static_cast<__underlying_type(T)>(val);
        }
    }
#endif


// #include <utility>
#include "nrf24_config.hpp"
#include "port_types_def.hpp"
// #include "spi.hpp"

extern void delayMs(uint32_t);
extern void delayUs(uint32_t);
// extern portType NRF24_CE_PORT;
// extern pinType NRF24_CE_PIN;

// extern portType NRF24_CSN_PORT;
// extern pinType NRF24_CSN_PIN;

#if defined(NRF24_SIMULATOR)
#include "nrf24_simulator.hpp"
inline uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    return nrf24::Nrf24::getInstance().transmitSpi(sendBuf, receiveBuf, cmd, len);
}
#else
extern uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len);
#endif

extern void setPinCE(uint_fast8_t state);

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
        delayMs(1);

        data[0] = SetRegister(Reg::EnRxAddr::erx_p0);
        // transmitSPI(data, data, WriteRegister(RegMap::EnRxAddr), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::EnRxAddr), 1);
        delayMs(1);

        data[0] = SetRegister(Reg::SetupAW::aw0, Reg::SetupAW::aw1);
        // transmitSPI(data, data, WriteRegister(RegMap::SetupAw), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::SetupAw), 1);
        delayMs(1);

        data[0] = 49;
        // transmitSPI(data, data, WriteRegister(RegMap::RFChannel), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::RFChannel), 1);
        delayMs(1);

        data[0] = SetRegister(Reg::RfSetup::rf_dr_low);
        // data[0] = SetRegister(Reg::RfSetup::rf_pwr1, Reg::RfSetup::rf_pwr2, Reg::RfSetup::rf_dr_low);
        // transmitSPI(data, data, WriteRegister(RegMap::RFSetup), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::RFSetup), 1);
        delayMs(1);

        // transmitSPI(rxAddress, data, WriteRegister(RegMap::RXAddressP0), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(rxAddress, data, WriteRegister(RegMap::RXAddressP0), 5);
        delayMs(1);

        // transmitSPI(txAddress, data, WriteRegister(RegMap::TXAddress), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(txAddress, data, WriteRegister(RegMap::TXAddress), 5);
        delayMs(1);

        data[0] = 32;
        // transmitSPI(data, data, WriteRegister(RegMap::RxPwP0), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::RxPwP0), 1);
        delayMs(1);


        data[0] = 0x6f;
        // transmitSPI(data, data, WriteRegister(RegMap::SetupRetr), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::SetupRetr), 1);
        delayMs(1);

        // Config for dynamic payload
        // data[0] = 0b00000011;
        data[0] = SetRegister(Reg::Dynpd::dpl_p0);
        // data[0] = 0x3f;
        // transmitSPI(data, data, WriteRegister(RegMap::DynPd), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::DynPd), 1);
        delayMs(1);

        // data[0] = 0b00000111;
        data[0] = SetRegister(Reg::Feature::en_dpl);
        // data[0] = 0;
        // transmitSPI(data, data, WriteRegister(RegMap::Feature), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::Feature), 1);
        delayMs(1);
        
        // data[0] = 0x73;
        // transmitSpiNrf24(data, data, WriteCmd(Commands::Activate), 1);

        // Power up and add crc
        data[0] = SetRegister(Reg::Config::crc0, Reg::Config::en_crc, Reg::Config::pwr_up, Reg::Config::prim_rx);
        // transmitSPI(data, data, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
        transmitSpiNrf24(data, data, WriteRegister(RegMap::Config), 1);
        delayMs(100);
    }

    inline void setPower(uint8_t state)
    {
        uint8_t data = 0;
        transmitSpiNrf24(nullptr, &data, ReadRegister(RegMap::Config), 1);

        if(state)
            data |= SetRegister(Reg::Config::pwr_up);
        else
            data &= ~SetRegister(Reg::Config::pwr_up);
        transmitSpiNrf24(&data, nullptr, WriteRegister(RegMap::Config), 1);
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
        transmitSpiNrf24(data, nullptr, std::to_underlying(Commands::WriteTxPayload), len);
    }

    inline void setToPTX()
    {
        setPinCE(0);
        // delayMs(5);
        uint8_t data = 0;
        transmitSpiNrf24(nullptr, &data, ReadRegister(RegMap::Config), 1);
        // delayMs(5);
        data &= ~SetRegister(Reg::Config::prim_rx);
        transmitSpiNrf24(&data, nullptr, WriteRegister(RegMap::Config), 1);
        // delayMs(5);

        setPinCE(1);
        delayUs(150);
        // delayMs(5);
    }

    inline void setToPRX()
    {
        setPinCE(0);
        // delayMs(5);
        uint8_t data = 0;
        transmitSpiNrf24(nullptr, &data, ReadRegister(RegMap::Config), 1);
        data |= SetRegister(Reg::Config::prim_rx);
        // delayMs(5);
        transmitSpiNrf24(&data, nullptr, WriteRegister(RegMap::Config), 1);
        // delayMs(5);
        setPinCE(1);
        delayUs(150);
        // delayMs(5);
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
        transmitSpiNrf24(nullptr, nullptr, std::to_underlying(command), 0);
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
        transmitSpiNrf24(nullptr, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1);
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

    /// @brief Check is nrf24 connected
    /// @return on success 1, otherwise 0
    inline int8_t isConnected()
    {
        // Value 0x0e is default value after reset in status register
        if(nrf24::getStatusReg() == 0x0e)
            return 1;

        return 0;
    }

    inline void resetStatusReg()
    {
        uint8_t resetReg = 0x70;
        transmitSpiNrf24(&resetReg, nullptr, nrf24::WriteRegister(nrf24::RegMap::Status), 1);
    }
}
