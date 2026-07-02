#pragma once
#include <cstdint>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <array>

#include "nrf24_config.hpp"

namespace nrf24
{
    class Nrf24
    {
        public:
        static Nrf24& getInstance()
        {
            std::lock_guard<std::mutex> guard(nrfMutex);
            static Nrf24 nrf24_;
            return nrf24_;
        }

        uint8_t transmitSpi(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
        {

            return memory[RegMap::Status];
        }

        private:
        Nrf24()
        {
            memory[RegMap::Config]      = 0b00001000;
            memory[RegMap::EnAA]        = 0b00111111;
            memory[RegMap::EnRxAddr]    = 0b00000011;
            memory[RegMap::SetupAw]     = 0b00000011;
            memory[RegMap::SetupRetr]   = 0b00000011;
            memory[RegMap::RFChannel]   = 0b00000010;
            memory[RegMap::RFSetup]     = 0b00001110;
            memory[RegMap::Status]      = 0b00001110;
            memory[RegMap::ObserveTx]   = 0b00000000;
            memory[RegMap::Rpd]         = 0b00000000;
            memory[RegMap::RXAddressP0] = 0b00000000;
            memory[RegMap::RXAddressP1] = 0b00000000;
            memory[RegMap::RXAddressP2] = 0b00000000;
            memory[RegMap::RXAddressP3] = 0b00000000;
            memory[RegMap::RXAddressP4] = 0b00000000;
            memory[RegMap::RXAddressP5] = 0b00000000;
            memory[RegMap::TXAddress]   = 0b00000000;
            memory[RegMap::RxPwP0]      = 0b00000000;
            memory[RegMap::RxPwP1]      = 0b00000000;
            memory[RegMap::RxPwP2]      = 0b00000000;
            memory[RegMap::RxPwP3]      = 0b00000000;
            memory[RegMap::RxPwP4]      = 0b00000000;
            memory[RegMap::RxPwP5]      = 0b00000000;
            memory[RegMap::FifoStatus]  = 0b00010001;
            memory[RegMap::DynPd]       = 0b00000000;
            memory[RegMap::Feature]     = 0b00000000;
        }

        void processSpiCommand(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
        {}
        
        inline static std::mutex nrfMutex;
        

        class Memory
        {
            public:
            uint8_t& operator[](nrf24::RegMap memAddress)
            {
                switch (memAddress)
                {
                case nrf24::RegMap::Config:
                case nrf24::RegMap::EnAA:
                case nrf24::RegMap::EnRxAddr:
                case nrf24::RegMap::SetupAw:
                case nrf24::RegMap::SetupRetr:
                case nrf24::RegMap::RFChannel:
                case nrf24::RegMap::RFSetup:
                case nrf24::RegMap::Status:
                case nrf24::RegMap::ObserveTx:
                case nrf24::RegMap::Rpd:
                case nrf24::RegMap::RXAddressP0:
                case nrf24::RegMap::RXAddressP1:
                case nrf24::RegMap::RXAddressP2:
                case nrf24::RegMap::RXAddressP3:
                case nrf24::RegMap::RXAddressP4:
                case nrf24::RegMap::RXAddressP5:
                case nrf24::RegMap::TXAddress:
                case nrf24::RegMap::RxPwP0:
                case nrf24::RegMap::RxPwP1:
                case nrf24::RegMap::RxPwP2:
                case nrf24::RegMap::RxPwP3:
                case nrf24::RegMap::RxPwP4:
                case nrf24::RegMap::RxPwP5:
                case nrf24::RegMap::FifoStatus:
                case nrf24::RegMap::DynPd:
                case nrf24::RegMap::Feature:
                    return rawMemory[static_cast<uint8_t>(memAddress)];
                default:
                    throw 1;
                    break;
                }
            }

            private:
            uint8_t rawMemory[0x1D + 1];
        };
        Memory memory;
        
        friend void threadTask(Nrf24& nrf);
    };


    inline void threadTask(Nrf24& nrf)
    {
        
    }

    static std::thread* nrfThread = nullptr; 

    [[gnu::always_inline]] inline void initSimulatorNRF24(const uint8_t* rxAddress, const uint8_t* txAddress)
    {
        if(nrfThread != nullptr)
            return;
        
        nrfThread = new std::thread(threadTask, std::ref(Nrf24::getInstance()));

        nrfThread->join();
    }
}