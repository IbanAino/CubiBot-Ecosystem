#include "LidarSpeedController.h"

namespace lidar {

SpeedController::SpeedController(uint8_t pwmPin)
    : pwm_(digitalPinToPinName(pwmPin))
{
}

void SpeedController::begin()
{
    pwm_.period_us(PWM_PERIOD_US);
    pwm_.write(0.50f);

    delay(150);
}

void SpeedController::setDutyCycle(float dutyCycle)
{
    if (dutyCycle < 0.0f) {
        dutyCycle = 0.0f;
    }
    else if (dutyCycle > 1.0f) {
        dutyCycle = 1.0f;
    }

    pwm_.write(dutyCycle);
}

}