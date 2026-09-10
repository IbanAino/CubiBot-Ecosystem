#include "LidarDriver.h"

lidar::Driver lidarDriver(Serial4);

void setup()
{
    Serial.begin(115200);

    lidarDriver.begin();
}

void loop()
{
    lidarDriver.process();

    lidar::Data data;

    if (lidarDriver.GetData(data)) {

        // Serial.print("Speed: ");
        // Serial.println(data.speed);

        // Serial.print("Timestamp: ");
        // Serial.println(data.timestamp);

        for (uint8_t i = 0; i < lidar::POINT_PER_PACK; ++i) {
            // Serial.print("Point ");
            // Serial.print(i);
            // Serial.print(" | angle: ");
            // Serial.print(data.points[i].angle);
            // Serial.print(" | distance: ");
            // Serial.print(data.points[i].distance);
            // Serial.print(" | intensity: ");
            // Serial.println(data.points[i].intensity);
            Serial.print(data.points[i].angle);
            Serial.print(",");
            Serial.println(data.points[i].distance);
        }
    }
}