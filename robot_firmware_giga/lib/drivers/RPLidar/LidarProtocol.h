#pragma once

#include <Arduino.h>

namespace lidar {

class ProtocolParser
{
public:
    static constexpr uint8_t HEADER = 0x54;
    static constexpr uint8_t VERLEN = 0x2C;
    static constexpr size_t FRAME_LENGTH = 47;

    ProtocolParser();

    void reset();

    // Reçoit un byte par appel
    // met le byte à la suite dans *frame, sauf si c'est un byte de début 0x54 0x2C
    // écrase les précédents données présentes dans *frame
    bool pushByte(uint8_t byte, uint8_t* frame);

private:
    uint8_t calculateCRC8(const uint8_t* data, size_t length);
    size_t rxIndex_;
};

}