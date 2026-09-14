#include "LidarDataConverter.h"

namespace lidar {

bool DataConverter::convert(const uint8_t* frame, Data& data)
{
    if (frame == nullptr) {
        return false;
    }


	// Serial.print("RAW: start bytes = 0x");
	// Serial.print(frame[4], HEX);
	// Serial.print(" 0x");
	// Serial.print(frame[5], HEX);

	// Serial.print(" | end bytes = 0x");
	// Serial.print(frame[42], HEX);
	// Serial.print(" 0x");
	// Serial.println(frame[43], HEX);


    // Start angle
    const uint16_t startAngle =
        static_cast<uint16_t>(frame[4]) |
        (static_cast<uint16_t>(frame[5]) << 8);

    // End angle
    const uint16_t endAngle =
        static_cast<uint16_t>(frame[42]) |
        (static_cast<uint16_t>(frame[43]) << 8);


	// Serial.print("Angles RAW: ");
	// Serial.print(startAngle / 100.0f);
	// Serial.print(" -> ");
	// Serial.print(endAngle / 100.0f);
	// Serial.print("  interval : ");
	// Serial.print((endAngle - startAngle) / 100.0f);

    // Speed
    data.speed =
        static_cast<uint16_t>(frame[2]) |
        (static_cast<uint16_t>(frame[3]) << 8);

	// Angles
	data.startAngle = startAngle;
	data.endAngle = endAngle;

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

	//Serial.println(data.points[0].angle);

    // Timestamp
    data.timestamp =
        static_cast<uint16_t>(frame[44]) |
        (static_cast<uint16_t>(frame[45]) << 8
	);

	// Serial.print("   Timestamp: ");
	// Serial.println(data.timestamp);

    return true;
}

uint16_t DataConverter::calculatePointAngle(
    uint16_t startAngle,
    uint16_t endAngle,
    uint8_t index)
{
    int32_t angleDifference =
        static_cast<int32_t>(endAngle) -
        static_cast<int32_t>(startAngle);

    // Choisir le chemin angulaire le plus court.
    if (angleDifference > static_cast<int32_t>(FULL_CIRCLE / 2)) {
        angleDifference -= FULL_CIRCLE;
    }
    else if (angleDifference < -static_cast<int32_t>(FULL_CIRCLE / 2)) {
        angleDifference += FULL_CIRCLE;
    }

    const int32_t angle =
        static_cast<int32_t>(startAngle) +
        (angleDifference * index) / (POINT_PER_PACK - 1);

    return static_cast<uint16_t>(
        (angle + FULL_CIRCLE) % FULL_CIRCLE
    );
}

} // namespace lidar