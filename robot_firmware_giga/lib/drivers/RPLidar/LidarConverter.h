#pragma once

#include <Arduino.h>
#include "LidarTypes.h"

namespace lidar {

class Converter
{
public:
    static constexpr uint16_t FULL_CIRCLE = 36000;
    //static constexpr uint8_t POINT_COUNT = 12;

    bool convert(const uint8_t* frame, Data& data);

private:
    uint16_t calculatePointAngle(
        uint16_t startAngle,
        uint16_t endAngle,
        uint8_t index);
};

} // namespace lidar