// PacketLidar = 88 octets

// ┌──────────────┬──────────────┬──────────────────────────────────────┐
// │ timestamp    │ speed        │ 12 points                            │
// │ uint16_t     │ uint16_t     │                                      │
// │ 2 octets     │ 2 octets     │ 12 × 7 octets = 84 octets           │
// └──────────────┴──────────────┴──────────────────────────────────────┘
//        0              2                       4                  88




#pragma once

#include <Arduino.h>

#include "LidarTypes.h"
#include "LidarProtocol.h"
#include "LidarDataConverter.h"
#include "LidarSpeedController.h"

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
	SpeedController speedController_;

    ProtocolParser parser_;
    DataConverter dataConverter_;

    uint8_t rxFrame_[ProtocolParser::FRAME_LENGTH];
    uint8_t readyFrame_[ProtocolParser::FRAME_LENGTH];

    bool frameAvailable_;
    bool started_;
};

} // namespace lidar