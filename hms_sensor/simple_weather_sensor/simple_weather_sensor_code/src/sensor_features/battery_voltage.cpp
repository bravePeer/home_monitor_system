#include "sensor_features/battery_voltage.hpp"

#include "spi.hpp"
#include "config.hpp"
#include "utils.hpp"
#include "platform_time.hpp"

void readBatteryVoltage(uint8_t* data)
{
    *batteryVoltageEnablePort |= (1<<batteryVoltageEnablePin); // Enable voltage measurement
    delayMs(10); // Wait for voltage to stabilize
    *batteryVoltageCsnPort &= ~(1<<batteryVoltageCsnPin);
    
    data[0] = transmitLowLevelSPI(&data[1], &data[1], 0, 1);
    *batteryVoltageCsnPort |= (1<<batteryVoltageCsnPin);
    delayMs(10); // Wait before good reading from ADC
    *batteryVoltageCsnPort &= ~(1<<batteryVoltageCsnPin);
    data[0] = transmitLowLevelSPI(&data[1], &data[1], 0, 1);

    *batteryVoltageCsnPort |= (1<<batteryVoltageCsnPin);
    *batteryVoltageEnablePort &= ~(1<<batteryVoltageEnablePin); // Disable voltage measurement
}