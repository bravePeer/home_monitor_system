#if defined(RP2040)
#include "spi.hpp"
extern "C"
{
    #include <pico/stdlib.h>
    #include <pico.h>
    #include <hardware/spi.h>
}

static const uint clkPin = 2;
static const uint mosiPin = 3;
static const uint misoPin = 4;

void intSPI(uint_fast16_t baud)
{
    gpio_init(clkPin);
    gpio_init(mosiPin);
    gpio_init(misoPin);
    gpio_set_function(clkPin, GPIO_FUNC_SPI);
    gpio_set_function(mosiPin, GPIO_FUNC_SPI);
    gpio_set_function(misoPin, GPIO_FUNC_SPI);

    spi_init(spi0, baud);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spi0, false);
}

uint8_t transmitLowLevelSPI(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    uint8_t data;
    spi_write_read_blocking(spi0, &cmd, &data, 1);
    if(len > 0)
        spi_write_read_blocking(spi0, sendBuf, receiveBuf, len);
    return data;
}
#endif