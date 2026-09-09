#include "DifferentialOdometryController.h"
#include <math.h>

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

DifferentialOdometryController::DifferentialOdometryController(RotaryIncrementalEncoder& leftEncoder,
                                                                   RotaryIncrementalEncoder& rightEncoder,
                                                                   DifferentialOdometry& odometry)
    : _leftEncoder(leftEncoder)
    , _rightEncoder(rightEncoder)
    , _odometry(odometry)
    , _previousLeftTicks(0)
    , _previousRightTicks(0)
    , _hasPreviousReading(false)
{
}

// -----------------------------------------------------------------------
// Core update
// -----------------------------------------------------------------------

void DifferentialOdometryController::update()
{
    const int32_t currentLeftTicks  = _leftEncoder.getTicks();
    const int32_t currentRightTicks = _rightEncoder.getTicks();

    // First call: just establish the baseline. Integrating a delta
    // against an undefined previous reading would produce a spurious
    // jump in the pose.
    if (!_hasPreviousReading) {
        _previousLeftTicks  = currentLeftTicks;
        _previousRightTicks = currentRightTicks;
        _hasPreviousReading = true;
        return;
    }

    int32_t deltaLeftTicks  = currentLeftTicks  - _previousLeftTicks;
    int32_t deltaRightTicks = currentRightTicks - _previousRightTicks;

    _previousLeftTicks  = currentLeftTicks;
    _previousRightTicks = currentRightTicks;

    const float deltaLeftMeters  = _ticksToMeters(deltaLeftTicks,  _leftEncoder.getTicksPerRev());
    const float deltaRightMeters = _ticksToMeters(deltaRightTicks, _rightEncoder.getTicksPerRev());

    _odometry.update(deltaLeftMeters, deltaRightMeters);
}

void DifferentialOdometryController::resetReferences()
{
    _hasPreviousReading = false; // forces a fresh baseline on next update()
}

// -----------------------------------------------------------------------
// Pass-through accessors
// -----------------------------------------------------------------------

float DifferentialOdometryController::getX() const     { return _odometry.getX(); }
float DifferentialOdometryController::getY() const     { return _odometry.getY(); }
float DifferentialOdometryController::getTheta() const { return _odometry.getTheta(); }

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

float DifferentialOdometryController::_ticksToMeters(int32_t deltaTicks, uint16_t ticksPerRev) const
{
    const float revolutions   = static_cast<float>(deltaTicks) / static_cast<float>(ticksPerRev);
    const float circumference = 2.0f * static_cast<float>(M_PI) * _odometry.getWheelRadius();
    return revolutions * circumference;
}
