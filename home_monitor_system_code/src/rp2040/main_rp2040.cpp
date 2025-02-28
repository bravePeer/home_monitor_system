#ifdef RP2040
// #include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/spi.h>

#include "rp2040/usb/usb_custom.h"

#include "spi.h"
#define ADDR(val) (*(volatile uint32_t *)(val))
#define NRF24_CSN_PORT ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET)
#define NRF24_CSN_PIN (uint8_t)6
#define NRF24_CE_PORT ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET)
#define NRF24_CE_PIN (uint8_t)7
#define NRF24_IRQ_PORT ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET)
#define NRF24_IRQ_PIN (uint8_t)8
#define DELAY_MS(val) sleep_ms(val)
#define DELAY_US(val) sleep_us(val)
#include "nRF24.h"

#include "sensor/sensor.hpp"

enum class LedState: uint32_t
{
    Off,
    OnRed,
    OnGreen,
    Invert
};

void setLedState(LedState ledState)
{
    const uint ledRedAnodePin = 17;
    const uint ledGreenAnodePin = 16;
    // gpio_set_drive_strength(ledGreenAnodePin, gpio_drive_strength::GPIO_DRIVE_STRENGTH_2MA);
    switch (ledState)
    {
    case LedState::Off:
        gpio_put(ledRedAnodePin, false);
        gpio_put(ledGreenAnodePin, false);
        break;
    case LedState::OnRed:
        gpio_put(ledGreenAnodePin, false);
        gpio_put(ledRedAnodePin, true);
        break;
    case LedState::OnGreen:
        gpio_put(ledGreenAnodePin, true);
        gpio_put(ledRedAnodePin, false);
        // gpio_set_drive_strength(ledGreenAnodePin, gpio_drive_strength::GPIO_DRIVE_STRENGTH_4MA);
        break;
    case LedState::Invert:

        break;
    }
}

volatile bool IrqStateNRF24 = false;

void gpioCallback(uint gpio, uint32_t event_mask)
{
    if(gpio == NRF24_IRQ_PIN && event_mask == GPIO_IRQ_EDGE_FALL)
    {
        IrqStateNRF24 = true;
    }
}

void processIrqStateNRF24()
{
    uint8_t dataToProcess[32];
    uint8_t dataCoutToProcess;

    switch(uint8_t status = nrf24::getStatusReg() & SetRegister(Reg::Status::max_rt, Reg::Status::tx_ds, Reg::Status::rx_dr))
        {
        case SetRegister(Reg::Status::rx_dr): // Data ready, received data
            transmitSPI(&dataCoutToProcess, &dataCoutToProcess, WriteCmd(Commands::ReadRxPayloadWidth), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
            
            setLedState(LedState::OnGreen);
            transmitSPI(dataToProcess, dataToProcess, WriteCmd(Commands::ReadRxPayload), dataCoutToProcess, &NRF24_CSN_PORT, NRF24_CSN_PIN);
            // memcpy(dataToProcess, usbData.data, dataCoutToProcess);
            if(sensor::processSensorPayload(dataToProcess, dataCoutToProcess) == -1)
                setLedState(LedState::OnRed);

            break;
        case SetRegister(Reg::Status::max_rt):

            break;
        case SetRegister(Reg::Status::tx_ds):

            break;
        default:
            break;
        }
        uint8_t resetReg = 0x70;
        transmitSPI(&resetReg, &resetReg, WriteRegister(RegMap::Status), 1, &NRF24_CSN_PORT, NRF24_CSN_PIN);
        // NRF24_CE_PORT |= (1<<NRF24_CE_PIN);
}

int main()
{
    stdio_init_all();

    gpio_init(5);
    gpio_init(17);
    gpio_init(16);
    gpio_init(NRF24_CE_PIN);
    gpio_init(NRF24_CSN_PIN);
    gpio_init(NRF24_IRQ_PIN);
    gpio_set_dir(NRF24_CE_PIN, true);
    gpio_set_dir(NRF24_CSN_PIN, true);
    gpio_set_dir(NRF24_IRQ_PIN, false);
    NRF24_CSN_PORT |= (1<<NRF24_CSN_PIN);


    gpio_set_dir(17, true);
    gpio_set_dir(16, true);

    setLedState(LedState::OnGreen);

    usb_device_init();

    usbData.dataLen = 10;

    for (size_t i = 0; i < usbData.dataLen; i++)
        usbData.data[i] = 'a';
    
   

    setLedState(LedState::OnRed);

    // NRF24_CE_PORT &= ~(1<<NRF24_CE_PIN); // TODO is this needed?
    intSPI(1000000);
    gpio_set_irq_callback(gpioCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(NRF24_IRQ_PIN, GPIO_IRQ_EDGE_FALL, true);

    nrf24::initnRF24();
    nrf24::setToPRX();
    NRF24_CE_PORT |= (1<<NRF24_CE_PIN);

    sleep_ms(2000);
    setLedState(LedState::OnGreen);
    sleep_ms(1000);
    setLedState(LedState::Off);

    bool receivedData = false;

    while (true) {
        if(receivedData)
        {
            receivedData = false;
            continue;
        }
        
        if(IrqStateNRF24)
        {
           IrqStateNRF24 = false; 
           processIrqStateNRF24();
        }
    }
}
#endif