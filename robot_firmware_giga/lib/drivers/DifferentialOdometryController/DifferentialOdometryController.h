#pragma once

#include <stdint.h>
#include "DifferentialOdometry.h"
#include "RotaryIncrementalEncoder.h"

/**
 * @brief Integration layer: connects DifferentialOdometry's pure math to
 *        two real RotaryIncrementalEncoder instances.
 *
 * Naming convention for this project: each odometry method has a pure-math
 * class ("<Method>Odometry") and a hardware integration class
 * ("<Method>OdometryController"). This is the integration layer for the
 * differential (two-wheel encoder) method specifically — other methods
 * (visual odometry, IMU-based, etc.) will get their own
 * "<Method>OdometryController" counterpart rather than reusing this name.
 *
 * This class owns no hardware — it holds references to encoders that
 * are constructed and configured elsewhere (same ownership philosophy
 * as VelocityMotorController). Each call to update() reads the current
 * cumulative tick count of both encoders, converts the tick delta since
 * the previous call into a wheel distance in meters, and feeds that into
 * the underlying DifferentialOdometry instance.
 *
 * Usage:
 *   RotaryIncrementalEncoder leftEncoder(2, 3, 11);
 *   RotaryIncrementalEncoder rightEncoder(18, 19, 11);
 *   DifferentialOdometry odometry(0.03f, 0.15f); // wheelRadius, wheelBase (m)
 *
 *   DifferentialOdometryController odomController(leftEncoder, rightEncoder, odometry);
 *   ...
 *   odomController.update(); // call at a steady rate, e.g. same as velocity loop
 *   float x = odomController.getX();
 */
class DifferentialOdometryController
{
public:
    /**
     * @param leftEncoder   Reference to the already-constructed left
     *                      wheel encoder (interrupts already attached).
     * @param rightEncoder  Reference to the already-constructed right
     *                      wheel encoder (interrupts already attached).
     * @param odometry      Reference to an already-constructed
     *                      DifferentialOdometry instance.
     */
    DifferentialOdometryController(RotaryIncrementalEncoder& leftEncoder,
                                     RotaryIncrementalEncoder& rightEncoder,
                                     DifferentialOdometry& odometry);

    /**
     * @brief Reads both encoders and integrates one odometry step.
     *
     * Call this at a steady rate (the same control period as your
     * velocity loop is a sensible choice — finer steps improve the
     * accuracy of the underlying Euler integration).
     *
     * The first call after construction (or after resetReferences())
     * only establishes the tick baseline and performs no integration,
     * since there is no previous reading to compute a delta from.
     */
    void update();

    /**
     * @brief Re-establishes the tick baseline without touching the pose.
     *
     * Call this if odometry updates are paused for a while (e.g. robot
     * idle) to avoid a large, spurious delta on the next update() call.
     */
    void resetReferences();

    // ------------------------------------------------------------------
    // Pass-through accessors (convenience — same data as the underlying
    // DifferentialOdometry instance)
    // ------------------------------------------------------------------

    float getX() const;
    float getY() const;
    float getTheta() const;

private:
    RotaryIncrementalEncoder& _leftEncoder;
    RotaryIncrementalEncoder& _rightEncoder;
    DifferentialOdometry&     _odometry;

    int32_t _previousLeftTicks;
    int32_t _previousRightTicks;
    bool    _hasPreviousReading;

    /** Converts a tick delta into a wheel-travelled distance in meters. */
    float _ticksToMeters(int32_t deltaTicks, uint16_t ticksPerRev) const;
};
