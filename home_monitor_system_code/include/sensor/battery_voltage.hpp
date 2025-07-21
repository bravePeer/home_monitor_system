#pragma once

#include "platform/attiny2313/config.hpp"
#include "spi.hpp"
#include "platform/attiny2313/utils.hpp"

void readBatteryVoltage(uint8_t* data)
{
    *batteryVoltageEnablePort |= (1<<batteryVoltageEnablePin); // Enable voltage measurement
    eDELAY_MS(10); // Wait for voltage to stabilize
    *batteryVoltagePort &= ~(1<<batteryVoltagePin);
    
    data[0] = transmitLowLevelSPI(&data[1], &data[1], 0, 1);
    *batteryVoltagePort |= (1<<batteryVoltagePin);
    eDELAY_MS(10); // Wait before good reading from ADC
    *batteryVoltagePort &= ~(1<<batteryVoltagePin);
    data[0] = transmitLowLevelSPI(&data[1], &data[1], 0, 1);

    *batteryVoltagePort |= (1<<batteryVoltagePin);
    *batteryVoltageEnablePort &= ~(1<<batteryVoltageEnablePin); // Disable voltage measurement
}