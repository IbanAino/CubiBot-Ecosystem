#pragma once

#include <Arduino.h>

namespace lidar {

enum class Status : uint8_t {
    Off,
    WaitingForData,
    Running,
    Timeout,
    RxOverflow,
    FrameError
};

struct Config {
    uint32_t baudRate = 230400;
    uint32_t timeoutMs = 500;
};

static constexpr size_t POINT_PER_PACK = 12;
static constexpr uint16_t FULL_CIRCLE = 36000;

// Nombre maximum de trames conservées simultanément.
// Réduire si la RAM est contrainte sur la cible.
static constexpr size_t MAX_FRAME_QUEUE_SIZE = 10;

struct __attribute__((packed)) Point
{
    //uint16_t angle;       // centièmes de degré
    uint16_t distance;    // mm
    uint8_t intensity;
};

struct __attribute__((packed)) Data
{
    uint16_t timestamp;
	uint16_t speed;
    uint16_t startAngle;
	uint16_t endAngle;
    Point points[POINT_PER_PACK];
};

} // namespace lidar