#pragma once
#if defined(WEATHER_RECEIVER)
#include <stdint.h>
#include "sensor/sensor_packet.hpp"
#include "sensor/sensor_data.hpp"
#include "utilities/ring_buffer.hpp"
#include "utilities/list.hpp"

constexpr int32_t maxKnownSensors = 10;
extern List<sensor::Sensor, maxKnownSensors> knownSensors;

constexpr int32_t maxPacketToSend = 10;
extern RingBuffer<sensorPacket::SensorPacketWithLen, maxPacketToSend> sendPacketRingBuffer;

void processSensorSends();

int processSensorPayload(const uint8_t* payload, uint8_t len);

void processIrqStateNRF24();


void initWirelessCommunicator();

#endif