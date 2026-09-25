#include "VelocityMotorController.h"
#include <Arduino.h>
#include <math.h>

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

VelocityMotorController::VelocityMotorController(RotaryIncrementalEncoder& encoder,
                                                     DCMotor& motor,
                                                     PIDController& pid,
                                                     StallWatchdog& stallWatchdog)
    : _encoder(encoder)
    , _motor(motor)
    , _pid(pid)
    , _stallWatchdog(stallWatchdog)
    , _targetVelocity(0.0f)
    , _measuredVelocity(0.0f)
	, _isDisabled(false)
    , _previousTicks(0)
    , _previousUpdateMicros(0)
    , _hasPreviousUpdate(false)
{
}

// -----------------------------------------------------------------------
// Setpoint
// -----------------------------------------------------------------------

void VelocityMotorController::setTargetVelocity(float targetRevPerSec)
{
    _targetVelocity = targetRevPerSec;
	_isDisabled = false; // an explicit velocity command is itself a re-engagement
}

float VelocityMotorController::getTargetVelocity() const
{
    return _targetVelocity;
}

float VelocityMotorController::getMeasuredVelocity() const
{
    return _measuredVelocity;
}

// -----------------------------------------------------------------------
// Control loop iteration
// -----------------------------------------------------------------------

void VelocityMotorController::update()
{
    if (_isDisabled) {
        return; // stop() was called: leave the motor and PID alone entirely,
                // including not tracking encoder ticks, until a new
                // setTargetVelocity() call re-engages the controller.
    }
	
	const uint32_t nowMicros = micros();
    const int32_t  currentTicks = _encoder.getTicks();

    // First call: just establish the baseline, no control action yet.
    // Acting on a dt of "zero elapsed time" would produce a meaningless
    // velocity measurement (division by near-zero).
    if (!_hasPreviousUpdate) {
        _previousTicks         = currentTicks;
        _previousUpdateMicros  = nowMicros;
        _hasPreviousUpdate     = true;
        return;
    }

    // Handle micros() overflow (wraps every ~70 minutes) safely:
    // unsigned subtraction wraps correctly regardless of overflow.
    const uint32_t elapsedMicros = nowMicros - _previousUpdateMicros;
    const float    dtSeconds     = static_cast<float>(elapsedMicros) / 1.0e6f;

    const int32_t deltaTicks = currentTicks - _previousTicks;

    _previousTicks        = currentTicks;
    _previousUpdateMicros = nowMicros;

    if (dtSeconds <= 0.0f) {
        return; // Guard against a degenerate call (e.g. called twice in the same micros() tick)
    }

    _measuredVelocity = _computeSignedVelocity(deltaTicks, dtSeconds);

    // --- Stall check: must run before the PID step, using this cycle's
    //     fresh measurement, so a stall is caught before another PID
    //     iteration can add to a stale integral term. ---
    _stallWatchdog.update(_targetVelocity, _measuredVelocity, dtSeconds);
    if (_stallWatchdog.isStalled()) {
        stop();
        return;
    }

    // --- PID: signed setpoint vs signed measurement ---
    const float pidOutput = _pid.compute(_targetVelocity, _measuredVelocity, dtSeconds);

    // --- Translate signed PID output into DCMotor's direction + magnitude API ---
    if (fabsf(pidOutput) < 1.0f) {
        // Below one PWM step: not worth spinning the motor, avoids buzzing
        // around zero when the PID output oscillates near the deadband.
        _motor.stop();
    } else if (pidOutput > 0.0f) {
        const uint8_t pwm = static_cast<uint8_t>(fminf(pidOutput, 255.0f));
        _motor.setSpeed(pwm);
        _motor.forward();
    } else {
        const uint8_t pwm = static_cast<uint8_t>(fminf(-pidOutput, 255.0f));
        _motor.setSpeed(pwm);
        _motor.backward();
    }
}

void VelocityMotorController::stop()
{
    _motor.stop(); // coast: both H-bridge IN pins LOW
	_disableAndResetPid();
}

void VelocityMotorController::brake()
{
    _motor.brake(); // electrodynamic brake: both H-bridge IN pins HIGH
    _disableAndResetPid();
}

bool VelocityMotorController::isDisabled() const
{
    return _isDisabled;
}

bool VelocityMotorController::isStalled() const
{
    return _stallWatchdog.isStalled();
}

void VelocityMotorController::clearStall()
{
    _stallWatchdog.reset();
	Serial.println("[VelocityMotorController] clearStall");
}

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

float VelocityMotorController::_computeSignedVelocity(int32_t deltaTicks, float dtSeconds) const
{
    const float ticksPerRev = static_cast<float>(_encoder.getTicksPerRev());
    const float revolutions = static_cast<float>(deltaTicks) / ticksPerRev;
    return revolutions / dtSeconds; // signed rev/s
}

void VelocityMotorController::_disableAndResetPid()
{
    _targetVelocity = 0.0f;
    _pid.reset();
    _hasPreviousUpdate = false; // force a fresh baseline if/when control resumes
    _isDisabled = true;
}