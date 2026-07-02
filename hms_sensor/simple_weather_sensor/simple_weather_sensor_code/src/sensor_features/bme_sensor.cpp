#include "sensor_features/bme_sensor.hpp"
// extern "C"
// {
//     #include <util/delay.h>
// }
#include "utils.hpp"

void initMeasurementSensor()
{
#if defined(USING_SENSOR_BME280)
    uint8_t data = (static_cast<uint8_t>(bme::ControlOversamplingTemperature::OversamplingX1) << 5) |
                    (static_cast<uint8_t>(bme::ControlOversamplingPressure::OversamplingX1) << 2) |
                    static_cast<uint8_t>(bme::ControlMode::Sleep);

    bme::writeReg(&data, bme::bme280::MemoryMap::CtrlMeas);

    data = static_cast<uint8_t>(bme::ControlOversamplingHumidity::OversamplingX1);
    bme::writeReg(&data, bme::bme280::MemoryMap::CtrlHum);
#elif defined(USING_SENSOR_BMP280)
    uint8_t data = (static_cast<uint8_t>(bme::ControlOversamplingTemperature::OversamplingX1) << 5) |
                    (static_cast<uint8_t>(bme::ControlOversamplingPressure::OversamplingX1) << 2) |
                    static_cast<uint8_t>(bme::ControlMode::Sleep);

    bme::writeReg(&data, bme::bmp280::MemoryMap::CtrlMeas);
#elif defined(USING_SENSOR_BME680)
// constexpr bme::BmeType bmeType = bme::BmeType::BME680;

#else
#error Select used sensor!
#endif
   
}

void doMeasurements()
{
    bme::BmeSelector<bmeType>::startForcedMeasurement();
    
    uint8_t counter = 20;
    do {
        if (counter == 0) 
            break;
        counter--;
        // _delay_ms(10);
        delayMs(10);
        
    } while (bme::BmeSelector<bmeType>::isBusy());
}