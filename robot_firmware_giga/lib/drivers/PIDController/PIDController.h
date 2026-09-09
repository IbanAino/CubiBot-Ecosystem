#pragma once

#include <stdint.h>

/**
 * @brief Generic PID controller — pure math, zero hardware dependency.
 *
 * This class has no knowledge of motors, encoders, or Arduino. It can be
 * reused for any closed-loop control problem: motor velocity, heading
 * hold, arm position, temperature regulation, etc.
 *
 * Time handling:
 *   The controller uses a variable timestep (dt) supplied by the caller
 *   on every compute() call, in seconds. This makes it robust to
 *   irregular call timing — the caller is responsible for measuring dt
 *   (typically via micros() on Arduino) and converting it to seconds.
 *
 * Anti-windup:
 *   The integral term is clamped to [-integralLimit, +integralLimit] to
 *   prevent unbounded growth when the actuator saturates (e.g. PWM stuck
 *   at 255 while the setpoint is still far away).
 *
 * Derivative filtering:
 *   The raw derivative amplifies measurement noise. A first-order
 *   low-pass filter is applied to the derivative term, controlled by
 *   derivativeFilterAlpha in [0, 1]:
 *     - alpha = 1.0 → no filtering (raw derivative)
 *     - alpha → 0.0 → heavy filtering (slow to react, very smooth)
 *
 * Usage:
 *   PIDController pid(2.0f, 0.5f, 0.1f, 100.0f, 0.2f);
 *   pid.setOutputLimits(-255.0f, 255.0f);
 *   ...
 *   float dtSeconds = (now - lastTime) / 1e6f;
 *   float output = pid.compute(setpoint, measurement, dtSeconds);
 */
class PIDController
{
public:
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /**
     * @param kp                     Proportional gain.
     * @param ki                     Integral gain.
     * @param kd                     Derivative gain.
     * @param integralLimit          Anti-windup clamp applied to the
     *                               accumulated integral term (absolute
     *                               value, same unit as the integral
     *                               contribution to the output).
     * @param derivativeFilterAlpha  Low-pass filter coefficient for the
     *                               derivative term, in [0, 1].
     *                               1.0 = no filtering, smaller = smoother.
     */
    PIDController(float kp,
                   float ki,
                   float kd,
                   float integralLimit,
                   float derivativeFilterAlpha);

    // ------------------------------------------------------------------
    // Core computation
    // ------------------------------------------------------------------

    /**
     * @brief Runs one PID iteration and returns the control output.
     *
     * @param setpoint     Desired target value.
     * @param measurement  Current measured value (same unit as setpoint).
     * @param dtSeconds    Elapsed time since the previous call, in seconds.
     *                     Must be > 0; if dtSeconds <= 0 the call is
     *                     ignored and the previous output is returned
     *                     unchanged (guards against a degenerate first
     *                     call or a stalled clock).
     * @return Control output, clamped to the configured output limits.
     */
    float compute(float setpoint, float measurement, float dtSeconds);

    /**
     * @brief Resets internal state (integral accumulator, previous error,
     *        filtered derivative). Call this whenever the controller is
     *        re-engaged after being idle, to avoid a stale integral term
     *        or a derivative spike from an old measurement.
     */
    void reset();

    // ------------------------------------------------------------------
    // Tuning
    // ------------------------------------------------------------------

    void setGains(float kp, float ki, float kd);
    void setIntegralLimit(float integralLimit);
    void setDerivativeFilterAlpha(float alpha);

    /**
     * @brief Sets the output clamp range. Defaults to [-INFINITY, +INFINITY]
     *        (i.e. unclamped) until called.
     */
    void setOutputLimits(float minOutput, float maxOutput);

    // ------------------------------------------------------------------
    // Introspection (useful for logging / tuning / debugging)
    // ------------------------------------------------------------------

    float getProportionalTerm() const;
    float getIntegralTerm() const;
    float getDerivativeTerm() const;
    float getLastOutput() const;

private:
    // Gains
    float _kp;
    float _ki;
    float _kd;

    // Anti-windup
    float _integralLimit;

    // Derivative filtering
    float _derivativeFilterAlpha;
    float _filteredDerivative;

    // Output clamping
    float _outputMin;
    float _outputMax;
    bool  _outputLimitsSet;

    // State across calls
    float _integral;
    float _previousError;
    bool  _hasPreviousError;

    // Last computed terms, kept for introspection
    float _lastProportionalTerm;
    float _lastIntegralTerm;
    float _lastDerivativeTerm;
    float _lastOutput;

    /** Clamps a value to [minVal, maxVal]. */
    static float _clamp(float value, float minVal, float maxVal);
};
