#pragma once

#include <stdint.h>
#include "PIDController.h"
#include "RotaryIncrementalEncoder.h"
#include "DCMotor.h"
#include "StallWatchdog.h"

/**
 * @brief Closed-loop velocity controller for one motor + encoder pair.
 *
 * This class is the integration layer: it owns a reference to a
 * RotaryIncrementalEncoder, a DCMotor, and a PIDController, and wires
 * them together every control cycle:
 *
 *   1. Read encoder speed (rev/s) and sign of motion from getTicks().
 *   2. Run the PID controller: setpoint = target rev/s, measurement = actual rev/s.
 *   3. Apply the PID output to the motor (direction + PWM magnitude).
 *
 * It does NOT own the encoder or motor objects — they are passed by
 * reference, so the same encoder/motor instances declared in your
 * sketch are reused. This keeps ownership and pin configuration in one
 * place (your main file) and avoids the controller silently
 * re-initialising hardware.
 *
 * Direction handling:
 *   DCMotor and RotaryIncrementalEncoder both work in unsigned/absolute
 *   terms (DCMotor::setSpeed takes 0-255, encoder getSpeed() returns a
 *   magnitude). This class is the one place where "signed velocity"
 *   exists: positive setpoint = forward, negative = backward.
 *
 * Usage:
 *   RotaryIncrementalEncoder leftEncoder(2, 3, 11);
 *   DCMotor leftMotor(9, 4, 5);
 *   PIDController leftPID(2.0f, 0.5f, 0.05f, 100.0f, 0.2f);
 *
 *   VelocityMotorController leftWheel(leftEncoder, leftMotor, leftPID);
 *   leftWheel.setTargetVelocity(3.0f); // rev/s, signed
 *   ...
 *   leftWheel.update(); // call this at a steady rate (e.g. every 20-50ms)
 */
class VelocityMotorController
{
public:
    /**
     * @param encoder  Reference to an already-constructed encoder
     *                 (interrupts must already be attached by the caller).
     * @param motor    Reference to an already-constructed motor.
     * @param pid      Reference to an already-constructed, already-tuned
     *                 PID controller dedicated to this wheel.
     * @param stallWatchdog  Reference to an already-constructed
     *                 StallWatchdog dedicated to this wheel. When it
     *                 detects a stall (e.g. motor power cut at the
     *                 hardware level, mechanical jam), update() will
     *                 automatically call stop() rather than let the PID
     *                 integral term wind up indefinitely.
     *
     * Each VelocityMotorController instance must be given its own
     * encoder, motor, PID, and stall watchdog instances — never share
     * one of these between two wheels, since their internal state is
     * per-axis.
     */
    VelocityMotorController(RotaryIncrementalEncoder& encoder,
                              DCMotor& motor,
                              PIDController& pid,
                              StallWatchdog& stallWatchdog);

    /**
     * @brief Sets the desired signed velocity in revolutions/second.
     *        Positive = forward, negative = backward, zero = stop.
     */
    void setTargetVelocity(float targetRevPerSec);

    /**
     * @brief Runs one control iteration: measure, compute PID, actuate.
     *
     * Call this at a roughly steady rate from your main loop. The PID
     * itself uses a measured dt internally, so jitter is tolerated, but
     * very irregular calling will degrade control quality.
     */
    void update();

    /** @brief Returns the last measured signed velocity (rev/s). */
    float getMeasuredVelocity() const;

    /** @brief Returns the current target velocity (rev/s). */
    float getTargetVelocity() const;

    /**
     * @brief Stops the motor, resets the PID internal state, and disables
     *        the controller.
     *
     * While disabled, update() does nothing at all — it will not run the
     * PID loop, will not react to an external force turning the wheel by
     * hand, and will not re-engage the motor on its own. This is the key
     * difference from setTargetVelocity(0.0f): a zero setpoint still
     * actively holds position against disturbances (normal closed-loop
     * behaviour), whereas stop() means "leave this wheel alone entirely"
     * — e.g. for safely manipulating the robot by hand.
     *
     * Disabling ends as soon as setTargetVelocity() is called again (see
     * its documentation) — stop() itself does not require a separate
     * acknowledgement, since it is an intentional command, not a fault.
     *
     * Motor behaviour: coasts to a stop (DCMotor::stop() — both H-bridge
     * IN pins LOW). The wheel is free to spin if turned by hand or by
     * residual momentum. Use brake() instead for an immediate, locked
     * stop.
     */
    void stop();
 
    /**
     * @brief Immediately and actively brakes the motor (DCMotor::brake()
     *        — both H-bridge IN pins HIGH, electrodynamic braking via the
     *        L298N), resets the PID internal state, and disables the
     *        controller exactly like stop().
     *
     * Use this for a hard/emergency stop where residual coasting motion
     * is undesirable, e.g. the robot must stop within a short, predictable
     * distance. Unlike stop(), the wheel actively resists being turned by
     * hand immediately after the call (the braking is electrical, not
     * just "no power") — though once update() is no longer running (the
     * controller is disabled), nothing keeps re-asserting the brake, so
     * the resistance is whatever the H-bridge's locked IN-HIGH state
     * provides on its own, not an active PID response.
     */
    void brake();

    /**
     * @brief Returns true if the controller is currently disabled
     *        (following a call to stop()) and update() is a no-op.
     */
    bool isDisabled() const;

    /**
     * @brief Returns true if the stall watchdog has detected a loss of
     *        actuation (e.g. power cut, mechanical jam). The motor is
     *        automatically stopped by update() when this becomes true.
     *
     * Stays true until clearStall() is called, even if the underlying
     * cause goes away — this avoids the motor silently resuming on its
     * own once power is restored.
     */
    bool isStalled() const;

    /**
     * @brief Clears the stall state, allowing update() to resume normal
     *        control on the next call. Call this only after confirming
     *        the actuator is safe to re-engage.
     */
    void clearStall();

private:
    RotaryIncrementalEncoder& _encoder;
    DCMotor&                  _motor;
    PIDController&            _pid;
    StallWatchdog&            _stallWatchdog;

    float _targetVelocity;     // signed, rev/s
    float _measuredVelocity;   // signed, rev/s
	bool  _isDisabled;

    int32_t _previousTicks;
    uint32_t _previousUpdateMicros;
    bool _hasPreviousUpdate;

    /**
     * @brief Computes signed velocity in rev/s from raw tick delta and dt.
     */
    float _computeSignedVelocity(int32_t deltaTicks, float dtSeconds) const;

    /**
     * @brief Shared logic between stop() and brake(): resets the target
     *        velocity and PID state, and disables the controller. Does
     *        NOT touch the motor itself — the caller is responsible for
     *        choosing _motor.stop() vs _motor.brake() before calling this.
     */
    void _disableAndResetPid();
};
