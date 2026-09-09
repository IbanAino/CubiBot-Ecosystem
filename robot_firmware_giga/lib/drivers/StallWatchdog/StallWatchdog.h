#pragma once

#include <stdint.h>

/**
 * @brief Generic stall / loss-of-actuation detector — pure logic, zero
 *        hardware dependency.
 *
 * Detects the situation where a command is being issued (non-zero target)
 * but the measured response stays below an expected threshold for too
 * long — e.g. a motor mechanically blocked, a power supply cut at the
 * hardware level, or any actuator that stops responding while still being
 * commanded.
 *
 * This class has no knowledge of motors, encoders, or PID controllers.
 * It only compares two numbers (command, measurement) over time. It is
 * reusable for any closed-loop actuator: wheel motors, an arm joint, a
 * servo, etc.
 *
 * Usage:
 *   StallWatchdog watchdog(0.1f, 1000); // minExpectedMagnitude, timeoutMs
 *   ...
 *   watchdog.update(targetVelocity, measuredVelocity, dtSeconds);
 *   if (watchdog.isStalled()) {
 *       // caller decides what to do: stop the motor, raise an alarm, etc.
 *   }
 */
class StallWatchdog
{
public:
    /**
     * @param minExpectedMagnitude  Minimum |measurement| considered as
     *                              "responding", whenever |command| is
     *                              above the same threshold. Below this,
     *                              the system is considered potentially
     *                              stalled (starts the timeout countdown).
     * @param stallTimeoutMs        Duration, in milliseconds, that the
     *                              measurement must stay below
     *                              minExpectedMagnitude — while the
     *                              command stays above it — before
     *                              isStalled() becomes true.
     */
    StallWatchdog(float minExpectedMagnitude, uint32_t stallTimeoutMs);

    /**
     * @brief Updates the watchdog state for one control cycle.
     *
     * @param command      The control input being issued (e.g. target
     *                      velocity). Only its magnitude matters.
     * @param measurement   The actual measured response (e.g. measured
     *                      velocity). Only its magnitude matters.
     * @param dtSeconds     Elapsed time since the previous update(), in
     *                      seconds.
     */
    void update(float command, float measurement, float dtSeconds);

    /**
     * @brief Returns true if the measurement has stayed below
     *        minExpectedMagnitude for longer than stallTimeoutMs while
     *        the command stayed above it.
     *
     * Stays true until reset() is called — the caller is expected to
     * take corrective action and explicitly clear the stall state,
     * rather than have it silently clear itself once the symptom goes
     * away (e.g. you do not want the motor to silently resume after a
     * hardware power cut is restored without an explicit re-engagement).
     */
    bool isStalled() const;

    /**
     * @brief Clears the stalled state and the internal timer.
     *        Call this once the underlying issue has been addressed and
     *        the actuator is being deliberately re-engaged.
     */
    void reset();

private:
    const float    _minExpectedMagnitude;
    const uint32_t _stallTimeoutMs;

    uint32_t _belowThresholdDurationMs;
    bool     _isStalled;
};
