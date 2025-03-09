#if defined(NATIVE) && !defined(UNIT_TEST)
#include <iostream>

#include <nrf24.hpp>
#include "spi.hpp"

pinType NRF24_CSN_PORT;
pinType NRF24_CSN_PIN;

using namespace std;

uint8_t transmitLowLevelSPI(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    cout << "Transmit: " <<hex<< static_cast<int>(cmd) << " ";
    for (uint8_t i = 0; i < len; i++)
    {
        cout << hex << static_cast<int>(sendBuf[i]) << " ";
    }
    cout<<endl;

    return len;
}

void eDELAY_MS(uint32_t a)
{
    cout<<"delay"<<endl;
}

int main()
{
    uint8_t rxAddress[5];
    uint8_t txAddress[5];
    nrf24::initnRF24(rxAddress, txAddress);
    cout << "Hello PIO!";
    printf("awdawd");
    return 0;
}
#endif