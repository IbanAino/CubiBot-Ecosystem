#pragma once

#include <Arduino.h>
#include "mbed.h"

namespace lidar {

class SpeedController
{
public:
    explicit SpeedController(uint8_t pwmPin);

    void begin();

    void setDutyCycle(float dutyCycle);

private:
    mbed::PwmOut pwm_;

    static constexpr uint32_t PWM_PERIOD_US = 1000;
};

}