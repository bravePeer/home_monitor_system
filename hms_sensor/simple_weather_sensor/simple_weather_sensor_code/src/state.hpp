#pragma once
#include <stdint.h>

enum class State : uint8_t
{
    // PwrUp,
    // PwrDown,
    WakeUp,
    ToIdle,
    ToIdleMaxRetr,
    Idle,
    ProcessReceivedData,
    DoMeasurements,
    SendPacket,
    ProcessIrq,
    WorkDone
    // Sleep
};

inline volatile State state;