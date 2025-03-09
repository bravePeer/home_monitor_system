#pragma once
#if defined(RP2040)
#include <hardware/regs/addressmap.h>
#include <hardware/regs/sio.h>
#include <pico/time.h>

// #define ADDR(val) (*(volatile uint32_t *)(val))
// #define NRF24_CSN_PORT ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET)
// #define NRF24_CSN_PIN (uint8_t)6
// #define NRF24_CE_PORT ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET)
// #define NRF24_CE_PIN (uint8_t)7
// #define NRF24_IRQ_PORT ADDR(SIO_BASE + SIO_GPIO_OUT_OFFSET)
// #define NRF24_IRQ_PIN (uint8_t)8
// #define DELAY_MS(val) sleep_ms(val)
// #define DELAY_US(val) sleep_us(val)

#endif