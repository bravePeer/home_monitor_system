#include "gpio.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "radio/radio.hpp"

constexpr uint32_t LedPin = 12;

volatile int32_t irqNrf24Flag = 0;

static void irqNrf24Handler(void* arg)
{
    // irqNrf24Flag = 1;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    notifyRadioTaskIsr(RadioTaskNotifyBits::NewDataReceived, &higherPriorityTaskWoken);
}

ErrorCode initGpio()
{

    gpio_config_t config = {};
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pin_bit_mask = (1 << LedPin) | (1 << nrf24CeGpio) | (1 << nrf24CsnGpio);
    gpio_config(&config);

    config.intr_type = GPIO_INTR_NEGEDGE;
    config.mode = GPIO_MODE_INPUT;
    config.pin_bit_mask = (1 << nrf24IrqGpio);
    gpio_config(&config);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(static_cast<gpio_num_t>(nrf24IrqGpio), irqNrf24Handler, nullptr);

    // Set default SPI CS level
    setLevel(nrf24CsnGpio, 1);
    setLevel(nrf24CeGpio, 1);

    return ErrorCode::Ok;
}

ErrorCode setLevel(uint32_t gpioNum, uint32_t state)
{
    gpio_set_level(static_cast<gpio_num_t>(gpioNum), state);
    return ErrorCode::Ok;
}

void setPinCE(uint_fast8_t state)
{
    gpio_set_level(static_cast<gpio_num_t>(nrf24CeGpio), state);
}
