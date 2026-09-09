#include "DCMotor.h"

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

DCMotor::DCMotor(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2)
    : _pinEnable(pinEnable)
    , _pinIN1(pinIN1)
    , _pinIN2(pinIN2)
    , _speed(0)
    , _direction(Direction::STOPPED)
{
    pinMode(_pinEnable, OUTPUT);
    pinMode(_pinIN1,    OUTPUT);
    pinMode(_pinIN2,    OUTPUT);

    // Safe state at startup : motor stopped, no voltage applied
    stop();
}

// -----------------------------------------------------------------------
// Speed
// -----------------------------------------------------------------------

void DCMotor::setSpeed(uint8_t speed)
{
    _speed = speed;
    analogWrite(_pinEnable, _speed);

    // Reaching speed 0 is semantically a stop
    if (_speed == 0) {
        _apply(LOW, LOW);
        _direction = Direction::STOPPED;
    }
}

uint8_t DCMotor::getSpeed() const
{
    return _speed;
}

// -----------------------------------------------------------------------
// Direction
// -----------------------------------------------------------------------

void DCMotor::forward()
{
    _direction = Direction::FORWARD;
    analogWrite(_pinEnable, _speed);
    _apply(HIGH, LOW);
}

void DCMotor::backward()
{
    _direction = Direction::BACKWARD;
    analogWrite(_pinEnable, _speed);
    _apply(LOW, HIGH);
}

void DCMotor::stop()
{
    _direction = Direction::STOPPED;
    _apply(LOW, LOW);
    // Leave PWM running — the L298N coast-stop is controlled by IN pins,
    // not by ENA/ENB, so speed is preserved for the next forward()/backward()
}

void DCMotor::brake()
{
    _direction = Direction::STOPPED;
    // Both IN pins HIGH → L298N shorts the motor windings (fast brake)
    _apply(HIGH, HIGH);
}

DCMotor::Direction DCMotor::getDirection() const
{
    return _direction;
}

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

void DCMotor::_apply(uint8_t in1, uint8_t in2) const
{
    digitalWrite(_pinIN1, in1);
    digitalWrite(_pinIN2, in2);
}
