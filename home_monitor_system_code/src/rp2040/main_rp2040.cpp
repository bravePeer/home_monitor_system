#ifdef RP2040
// #include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/spi.h>

#include "rp2040/usb/usb_custom.h"
#include "rp2040/config.hpp"
#include "spi.hpp"

// #include "nRF24.h"

#include "rp2040/wireless_communicator/wireless_communicator.hpp"

#include "rp2040/led.hpp"

#define ADDR(val) (*(volatile uint32_t *)(val))

portType NRF24_CSN_PORT = &ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET);
pinType NRF24_CSN_PIN = 6;
portType NRF24_CE_PORT = &ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET);
pinType NRF24_CE_PIN = 7;
portType NRF24_IRQ_PORT = &ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET);
pinType NRF24_IRQ_PIN = 8;

uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    *NRF24_CSN_PORT &= (~(1<<NRF24_CSN_PIN));
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    *NRF24_CSN_PORT |= (1<<NRF24_CSN_PIN);
    return ret;
    // transmitSPI(sendBuf, receiveBuf, cmd, 1, NRF24_CSN_PORT, NRF24_CSN_PIN);
}

void eDELAY_MS(uint32_t val) 
{
    sleep_ms(val);
}

void eDELAY_US(uint32_t val) 
{
    sleep_us(val);
}

volatile bool IrqStateNRF24 = false;

void gpioCallback(uint gpio, uint32_t event_mask)
{
    if(gpio == NRF24_IRQ_PIN && event_mask == GPIO_IRQ_EDGE_FALL)
    {
        IrqStateNRF24 = true;
    }
}




int main()
{
    stdio_init_all();
    initLedState();
    gpio_init(5);

    gpio_init(NRF24_CE_PIN);
    gpio_init(NRF24_CSN_PIN);
    gpio_init(NRF24_IRQ_PIN);
    gpio_set_dir(NRF24_CE_PIN, true);
    gpio_set_dir(NRF24_CSN_PIN, true);
    gpio_set_dir(NRF24_IRQ_PIN, false);
    *NRF24_CSN_PORT |= (1 << NRF24_CSN_PIN);

    setLedState(LedState::OnRed);

    usb_device_init();


    intSPI(5000000);
    gpio_set_irq_callback(gpioCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(NRF24_IRQ_PIN, GPIO_IRQ_EDGE_FALL, true);

    initWirelessCommunicator();

    blinkLed(LedState::OnGreen, 2, 500);

    // bool receivedData = false;

    while (true) {
        // if(receivedData)
        // {
        //     receivedData = false;
        //     continue;
        // }
        
        if(IrqStateNRF24)
        {
           IrqStateNRF24 = false; 
           processIrqStateNRF24();
        }
        // DELAY_MS(100);
        processSensorSends();
    }
}
#endif