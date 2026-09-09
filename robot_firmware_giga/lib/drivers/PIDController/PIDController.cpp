#include "PIDController.h"
#include <math.h>

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

PIDController::PIDController(float kp,
                               float ki,
                               float kd,
                               float integralLimit,
                               float derivativeFilterAlpha)
    : _kp(kp)
    , _ki(ki)
    , _kd(kd)
    , _integralLimit(fabsf(integralLimit))
    , _derivativeFilterAlpha(derivativeFilterAlpha)
    , _filteredDerivative(0.0f)
    , _outputMin(-INFINITY)
    , _outputMax(INFINITY)
    , _outputLimitsSet(false)
    , _integral(0.0f)
    , _previousError(0.0f)
    , _hasPreviousError(false)
    , _lastProportionalTerm(0.0f)
    , _lastIntegralTerm(0.0f)
    , _lastDerivativeTerm(0.0f)
    , _lastOutput(0.0f)
{
}

// -----------------------------------------------------------------------
// Core computation
// -----------------------------------------------------------------------

float PIDController::compute(float setpoint, float measurement, float dtSeconds)
{
    // Guard against a degenerate or stalled timestep. Returning the
    // previous output avoids a division-by-zero in the derivative term
    // and avoids corrupting the integral with a bogus dt.
    if (dtSeconds <= 0.0f) {
        return _lastOutput;
    }

    const float error = setpoint - measurement;

    // --- Proportional term ---
    _lastProportionalTerm = _kp * error;

    // --- Integral term, with anti-windup clamping ---
    _integral += error * dtSeconds;
    _integral = _clamp(_integral, -_integralLimit, _integralLimit);
    _lastIntegralTerm = _ki * _integral;

    // --- Derivative term, with low-pass filtering ---
    // On the very first call there is no previous error to derive from;
    // treat the raw derivative as zero to avoid a spurious spike.
    float rawDerivative = 0.0f;
    if (_hasPreviousError) {
        rawDerivative = (error - _previousError) / dtSeconds;
    }

    // First-order low-pass filter:
    //   filtered = alpha * raw + (1 - alpha) * filtered_previous
    _filteredDerivative = _derivativeFilterAlpha * rawDerivative
                         + (1.0f - _derivativeFilterAlpha) * _filteredDerivative;

    _lastDerivativeTerm = _kd * _filteredDerivative;

    _previousError    = error;
    _hasPreviousError = true;

    // --- Sum and clamp output ---
    float output = _lastProportionalTerm + _lastIntegralTerm + _lastDerivativeTerm;

    if (_outputLimitsSet) {
        output = _clamp(output, _outputMin, _outputMax);
    }

    _lastOutput = output;
    return output;
}

void PIDController::reset()
{
    _integral           = 0.0f;
    _previousError      = 0.0f;
    _hasPreviousError   = false;
    _filteredDerivative = 0.0f;

    _lastProportionalTerm = 0.0f;
    _lastIntegralTerm     = 0.0f;
    _lastDerivativeTerm   = 0.0f;
    _lastOutput           = 0.0f;
}

// -----------------------------------------------------------------------
// Tuning
// -----------------------------------------------------------------------

void PIDController::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setIntegralLimit(float integralLimit)
{
    _integralLimit = fabsf(integralLimit);
    _integral = _clamp(_integral, -_integralLimit, _integralLimit);
}

void PIDController::setDerivativeFilterAlpha(float alpha)
{
    _derivativeFilterAlpha = _clamp(alpha, 0.0f, 1.0f);
}

void PIDController::setOutputLimits(float minOutput, float maxOutput)
{
    _outputMin = minOutput;
    _outputMax = maxOutput;
    _outputLimitsSet = true;
}

// -----------------------------------------------------------------------
// Introspection
// -----------------------------------------------------------------------

float PIDController::getProportionalTerm() const { return _lastProportionalTerm; }
float PIDController::getIntegralTerm() const     { return _lastIntegralTerm; }
float PIDController::getDerivativeTerm() const   { return _lastDerivativeTerm; }
float PIDController::getLastOutput() const       { return _lastOutput; }

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

float PIDController::_clamp(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}
