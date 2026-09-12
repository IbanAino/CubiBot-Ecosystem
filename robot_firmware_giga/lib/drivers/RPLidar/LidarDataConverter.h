// Les données arrivent sous la forme de 12 mesures de distance avec un angle de début et un angle de fin
// Il faut donc dispacher uniformément ces douze mesures entre les deux angles pour leur attribuer un angle à chacune
//
// Les distance dans la trames sont stockées sur deux octets (un octet fort et un octet faible)
// Un cast est opéré pour ne plus stocker une distance que sur un seul uint16_t
//
// Index C/C++    Documentation

// frame[0]       Header
// frame[1]       VerLen

// frame[2]       Speed LOW
// frame[3]       Speed HIGH

// frame[4]       Start angle LOW
// frame[5]       Start angle HIGH

// frame[6]       Point 0 distance LOW
// frame[7]       Point 0 distance HIGH
// frame[8]       Point 0 intensity

// frame[9]       Point 1 distance LOW
// frame[10]      Point 1 distance HIGH
// frame[11]      Point 1 intensity

// ...

// frame[39]      Point 11 distance LOW
// frame[40]      Point 11 distance HIGH
// frame[41]      Point 11 intensity

// frame[42]      End angle LOW
// frame[43]      End angle HIGH

// frame[44]      Timestamp LOW
// frame[45]      Timestamp HIGH

// frame[46]      CRC

#pragma once

#include <Arduino.h>
#include "LidarTypes.h"

namespace lidar {

class DataConverter
{
public:
    bool convert(const uint8_t* frame, Data& data);

private:
    uint16_t calculatePointAngle(
        uint16_t startAngle,
        uint16_t endAngle,
        uint8_t index);
};

} // namespace lidar