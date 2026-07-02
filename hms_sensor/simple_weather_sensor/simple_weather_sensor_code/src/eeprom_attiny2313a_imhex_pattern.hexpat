struct SimpleWeatherSensor
{
    u8 radioAddress[5];
    u32 sensorIdentifier;
    u32 softwareVersion;
    u32 hardwareVersion;
    u32 sensorType;
    u8 maxDataPacketsCount;
    u8 maxCalibPacketsCount;
    // Resistor dividers are not implemented
    u16 resistorDividerR1;
    u16 resistorDividerR2;
};

SimpleWeatherSensor sws @ 0x0;