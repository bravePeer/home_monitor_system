#if defined(NATIVE) && !defined(UNIT_TEST)
#include <iostream>

// #include <nrf24.hpp>
// #include "spi.hpp"

// pinType NRF24_CSN_PORT;
// pinType NRF24_CSN_PIN;

using namespace std;

// void eDELAY_MS(uint32_t a)
// {
    // cout<<"delay"<<endl;
// }

#include "packet/packetstr.hpp"

int main()
{
    char arr[256];
    for (size_t i = 0; i < 256; i++)
    {
        arr[i] = 0;
    }
    
    sensorPacket::SensorPacket packet;
    packet.General.header.direction = sensorPacket::PacketDirection::Report;
    packet.General.header.errorFlag = sensorPacket::PacketError::Error;
    packet.General.header.type = sensorPacket::PacketType::SensorData1;
    packet.General.crc = 0xaa;
    sensorPacket::bytesToHexStr(packet.raw, 32, arr);
    std::cout << arr << std::endl;


    std::cout << sensorPacket::packetToStr(&packet, 32).buf << std::endl;
    std::cout << sensorPacket::packetToStrInfo(&packet, 32).buf << std::endl;
    // uint8_t rxAddress[5];
    // uint8_t txAddress[5];
    // nrf24::initnRF24(rxAddress, txAddress);
    // cout << "Hello PIO!";
    // printf("awdawd");
    return 0;
}
#endif