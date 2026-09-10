#include "LidarConverter.h"

namespace lidar {

bool Converter::convert(const uint8_t* frame, Data& data)
{
    if (frame == nullptr) {
        return false;
    }

    // Start angle
    const uint16_t startAngle =
        static_cast<uint16_t>(frame[4]) |
        (static_cast<uint16_t>(frame[5]) << 8);

    // End angle
    const uint16_t endAngle =
        static_cast<uint16_t>(frame[42]) |
        (static_cast<uint16_t>(frame[43]) << 8);

    // Speed
    data.speed =
        static_cast<uint16_t>(frame[2]) |
        (static_cast<uint16_t>(frame[3]) << 8);

    // Calculate all points
    for (uint8_t i = 0; i < POINT_PER_PACK; ++i) {
        const uint8_t index = 6 + (i * 3);

        data.points[i].distance =
            static_cast<uint16_t>(frame[index]) |
            (static_cast<uint16_t>(frame[index + 1]) << 8);

        data.points[i].intensity = frame[index + 2];

        data.points[i].angle =
            calculatePointAngle(startAngle, endAngle, i);
    }

    // Timestamp
    data.timestamp =
        static_cast<uint16_t>(frame[44]) |
        (static_cast<uint16_t>(frame[45]) << 8);

    return true;
}

uint16_t Converter::calculatePointAngle(
    uint16_t startAngle,
    uint16_t endAngle,
    uint8_t index)
{
    uint16_t angleRange;

    if (endAngle >= startAngle) {
        angleRange = endAngle - startAngle;
    } else {
        angleRange = (FULL_CIRCLE - startAngle) + endAngle;
    }

    // 12 points means 11 intervals between start and end.
    const uint32_t angle =
        static_cast<uint32_t>(startAngle) +
        (static_cast<uint32_t>(angleRange) * index) / (POINT_PER_PACK - 1);

    return static_cast<uint16_t>(angle % FULL_CIRCLE);
}

} // namespace lidar