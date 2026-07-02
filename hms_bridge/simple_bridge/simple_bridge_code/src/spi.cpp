// #include "platform/general/platform_spi.hpp"
#include "config.hpp"
#include "spi.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "gpio.hpp"

constexpr int SpiPinMiso = 2;
constexpr int SpiPinMosi = 7;
constexpr int SpiPinSclk = 6;

static spi_device_handle_t radioHandle;

// ErrorCode initSpi()
void intSPI(uint_fast16_t baud)
{
    // Init SPI
    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num = SpiPinMosi;
    busConfig.miso_io_num = SpiPinMiso;
    busConfig.sclk_io_num = SpiPinSclk;
    busConfig.quadwp_io_num = -1;
    busConfig.quadhd_io_num = -1;
    // busConfig.data4_io_num = -1;
    // busConfig.data5_io_num = -1;
    // busConfig.data6_io_num = -1;
    // busConfig.data7_io_num = -1;
    // busConfig.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;
   
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_DISABLED));

    spi_device_interface_config_t radioDeviceConfig = {};
    radioDeviceConfig.command_bits = 0;
    radioDeviceConfig.address_bits = 0;
    radioDeviceConfig.dummy_bits = 0;
    radioDeviceConfig.mode = 0;
    radioDeviceConfig.clock_source = SPI_CLK_SRC_DEFAULT;
    // radioDeviceConfig.duty_cycle_pos = 0;
    // radioDeviceConfig.cs_ena_pretrans = 0;
    // radioDeviceConfig.cs_ena_posttrans = 0;
    radioDeviceConfig.clock_speed_hz = 1000000;
    radioDeviceConfig.spics_io_num = -1;
    radioDeviceConfig.queue_size = 1;
        // .spics_io_num = 
    

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &radioDeviceConfig, &radioHandle));


    // Init SPI CS pins

    // return ErrorCode::Ok;
}

uint8_t transmitLowLevelSPI(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    uint8_t ret = 0;
    spi_transaction_t transactionCmd { };
    transactionCmd.length = 8;
    transactionCmd.rxlength = 8;
    transactionCmd.rx_buffer = &ret;
    transactionCmd.tx_buffer = &cmd;

    ESP_ERROR_CHECK(spi_device_polling_transmit(radioHandle, &transactionCmd));
    
    if(sendBuf == nullptr && receiveBuf == nullptr)
        return ret;

    spi_transaction_t transaction { };
    transaction.length = len * 8;
    transaction.rxlength = len * 8;
    transaction.rx_buffer = receiveBuf;
    transaction.tx_buffer = sendBuf;

    ESP_ERROR_CHECK(spi_device_polling_transmit(radioHandle, &transaction));

    return ret;
}

uint8_t transmitSpiNrf24(const uint8_t* sendBuf, uint8_t* receiveBuf, const uint8_t cmd, const uint8_t len)
{
    setLevel(nrf24CsnGpio, 0);
    uint8_t ret = transmitLowLevelSPI(sendBuf, receiveBuf, cmd, len);
    setLevel(nrf24CsnGpio, 1);
    
    return ret;
}