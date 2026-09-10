#pragma once

#include <Arduino.h>

#include "LidarTypes.h"
#include "LidarProtocol.h"
#include "LidarConverter.h"

namespace lidar {

class Driver
{
public:
    Driver(HardwareSerial& serial);

    void begin();
    void stop();
    void process();

    bool GetData(Data& data);

private:
    HardwareSerial& serial_;

    ProtocolParser parser_;
    Converter converter_;

    uint8_t frame_[ProtocolParser::FRAME_LENGTH];

    bool frameAvailable_;
    bool started_;
};

} // namespace lidar