#include "nrf24.hpp"
#include "nrf24_config.hpp"
#include "spi.hpp"

namespace nrf24
{
    // void initnRF24(const uint8_t* rxAddress, const uint8_t* txAddress) 
    // {
    //     // IRQ_DDR &= ~(1<<irqPIN);
    //     // CE_DDR |= (1<<cePIN);

    //     //Send address
    //     uint8_t data[5];

    //     data[0] = SetRegister(Reg::EnAA::enaa_p0);
    //     // transmitSPI(data, data, WriteRegister(RegMap::EnAA), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::EnAA), 1);
    //     eDELAY_MS(1);

    //     data[0] = SetRegister(Reg::EnRxAddr::erx_p0);
    //     // transmitSPI(data, data, WriteRegister(RegMap::EnRxAddr), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::EnRxAddr), 1);
    //     eDELAY_MS(1);

    //     data[0] = SetRegister(Reg::SetupAW::aw0, Reg::SetupAW::aw1);
    //     // transmitSPI(data, data, WriteRegister(RegMap::SetupAw), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::SetupAw), 1);
    //     eDELAY_MS(1);

    //     data[0] = 76;
    //     // transmitSPI(data, data, WriteRegister(RegMap::RFChannel), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::RFChannel), 1);
    //     eDELAY_MS(1);

    //     // data[0] = SetRegister(Reg::RfSetup::rf_dr_low);
    //     data[0] = SetRegister(Reg::RfSetup::rf_pwr1, Reg::RfSetup::rf_pwr2, Reg::RfSetup::rf_dr_low);
    //     // transmitSPI(data, data, WriteRegister(RegMap::RFSetup), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::RFSetup), 1);
    //     eDELAY_MS(1);

    //     // transmitSPI(rxAddress, data, WriteRegister(RegMap::RXAddressP0), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(rxAddress, data, WriteRegister(RegMap::RXAddressP0), 5);
    //     eDELAY_MS(1);

    //     // transmitSPI(txAddress, data, WriteRegister(RegMap::TXAddress), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(txAddress, data, WriteRegister(RegMap::TXAddress), 5);
    //     eDELAY_MS(1);

    //     data[0] = 32;
    //     // transmitSPI(data, data, WriteRegister(RegMap::RxPwP0), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::RxPwP0), 1);
    //     eDELAY_MS(1);


    //     data[0] = 0x2f;
    //     // transmitSPI(data, data, WriteRegister(RegMap::SetupRetr), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::SetupRetr), 1);
    //     eDELAY_MS(1);

    //     // Config for dynamic payload
    //     // data[0] = 0b00000011;
    //     data[0] = SetRegister(Reg::Dynpd::dpl_p0);
    //     // data[0] = 0x3f;
    //     // transmitSPI(data, data, WriteRegister(RegMap::DynPd), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::DynPd), 1);
    //     eDELAY_MS(1);

    //     // data[0] = 0b00000111;
    //     data[0] = SetRegister(Reg::Feature::en_dpl);
    //     // data[0] = 0;
    //     // transmitSPI(data, data, WriteRegister(RegMap::Feuture), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::Feuture), 1);
    //     eDELAY_MS(1);
        
    //     // Power up and add crc
    //     data[0] = SetRegister(Reg::Config::crc0, Reg::Config::en_crc, Reg::Config::pwr_up, Reg::Config::prim_rx);
    //     // transmitSPI(data, data, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     transmitSpiNrf24(data, data, WriteRegister(RegMap::Config), 1);
    //     eDELAY_MS(100);
    // }

    // void setRxAddress(const uint8_t* rxAddress)
    // {
    //     transmitSPI(rxAddress, nullptr, WriteRegister(RegMap::RXAddressP0), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }

    // void setTxAddress(const uint8_t* txAddress)
    // {
    //     transmitSPI(txAddress, nullptr, WriteRegister(RegMap::TXAddress), 5, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }

    // void sendData(const uint8_t* data, uint8_t len)
    // {
    //     // transmitSPI(nullptr,nullptr, static_cast<uint8_t>(Commands::FlushTx),0);
    //     // eDELAY_US(50);
    //     // Send to TX buffer
    //     transmitSPI(data, nullptr, static_cast<uint8_t>(Commands::WriteTxPayload), len, NRF24_CSN_PORT, NRF24_CSN_PIN);

    //     // Set PRIM_RX = 0

    //     // Set CE = 1
    //     // DELAY_MS(10);
    //     // CE_PORT |= (1<<cePIN);
    //     // eDELAY_US(20);
    //     // CE_PORT &= ~(1<<cePIN);
    //     // DELAY_MS(10);  
    // }

    // void setToPTX()
    // {
    //     *NRF24_CE_PORT &= (~(1<<NRF24_CE_PIN));
    //     eDELAY_MS(5);
    //     uint8_t data = 0;
    //     transmitSPI(nullptr, &data, ReadRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     eDELAY_MS(5);
    //     data &= ~SetRegister(Reg::Config::prim_rx);
    //     transmitSPI(&data, nullptr, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     eDELAY_MS(5);
    //     *NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
    //     eDELAY_MS(5);
    // }

    // void setToPRX()
    // {
    //     *NRF24_CE_PORT &= (~(1<<NRF24_CE_PIN));
    //     eDELAY_MS(5);
    //     uint8_t data = 0;
    //     transmitSPI(nullptr, &data, ReadRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     data |= SetRegister(Reg::Config::prim_rx);
    //     eDELAY_MS(5);
    //     transmitSPI(&data, nullptr, WriteRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     eDELAY_MS(5);
    //     *NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
    //     eDELAY_MS(5);
    // }

    // uint8_t getStatusReg()
    // {
    //     return transmitSPI(nullptr, nullptr, Cmd(Commands::Nop), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }

    // uint8_t getConfig()
    // {
    //     uint8_t data = 0;
    //     transmitSPI(&data, &data, ReadRegister(RegMap::Config), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     return data;
    // }

    // void writeCommand(Commands command)
    // {
    //     transmitSPI(nullptr, nullptr, static_cast<uint8_t>(command), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }

    // /// @brief 
    // /// @param data 32 byte buffer
    // /// @return read bytes count
    // uint8_t readRxPayload(uint8_t* data)
    // {
    //     uint8_t dataCount = 0;
    //     transmitSPI(&dataCount, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     if(dataCount > 32)
    //     {
    //         transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //         return 0;
    //     }
    //     transmitSPI(data, data, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCount, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     return dataCount;
    // }

    // uint8_t getRxPayloadCount()
    // {
    //     uint8_t dataCount = 0;
    //     transmitSPI(&dataCount, &dataCount, nrf24::WriteCmd(nrf24::Commands::ReadRxPayloadWidth), 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
    //     return dataCount;
    // }

    // void readRxPayloadRaw(uint8_t* data, uint8_t dataCount)
    // {
    //     transmitSPI(data, data, nrf24::WriteCmd(nrf24::Commands::ReadRxPayload), dataCount, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }

    
    // void flushTx()
    // {
    //     transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushTx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }

    // void flushRx()
    // {
    //     transmitSPI(nullptr, nullptr, nrf24::WriteCmd(nrf24::Commands::FlushRx), 0, NRF24_CSN_PORT, NRF24_CSN_PIN);
    // }
}
