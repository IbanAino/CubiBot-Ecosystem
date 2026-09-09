#pragma once

/**
 * @brief Differential-drive odometry — pure math, zero hardware dependency.
 *
 * This class has no knowledge of encoders, motors, or Arduino. It only
 * knows the classic differential-drive kinematics: given the distance
 * travelled by the left and right wheels since the last update, it
 * integrates the robot's pose (x, y, theta) in a fixed global frame.
 *
 * Convention:
 *   - x, y in meters, global frame, origin at the pose when reset() was
 *     last called (or at construction).
 *   - theta in radians, 0 = facing the +x axis, positive = counter-clockwise,
 *     normalized to [-pi, pi] after every update.
 *   - Positive wheel distance = that wheel moved forward.
 *
 * This is a first-order (Euler) integration: accurate for small steps
 * (i.e. called at a steady, reasonably fast rate — the same control
 * period as your velocity loop is a good choice). It assumes pure
 * rolling without slippage; wheel slip will silently bias the result.
 *
 * Usage:
 *   DifferentialOdometry odom(0.03f, 0.15f); // wheelRadius, wheelBase, meters
 *   ...
 *   odom.update(deltaLeftMeters, deltaRightMeters);
 *   float x = odom.getX();
 *   float heading = odom.getTheta();
 */
class DifferentialOdometry
{
public:
    /**
     * @param wheelRadiusMeters  Radius of the wheels, in meters.
     *                           (Kept for convenience/documentation; the
     *                           update() method takes distances already
     *                           converted to meters, so this value is not
     *                           used internally — see integration layer.)
     * @param wheelBaseMeters    Distance between the two wheel contact
     *                           points (the track width), in meters.
     */
    DifferentialOdometry(float wheelRadiusMeters, float wheelBaseMeters);

    /**
     * @brief Integrates one odometry step from wheel displacement.
     *
     * @param deltaLeftMeters   Distance travelled by the left wheel since
     *                          the previous update, in meters (signed:
     *                          negative means that wheel moved backward).
     * @param deltaRightMeters  Distance travelled by the right wheel since
     *                          the previous update, in meters (signed).
     */
    void update(float deltaLeftMeters, float deltaRightMeters);

    /**
     * @brief Resets the pose to the origin (0, 0, 0).
     *
     * Use this to redefine the current position/heading as the new
     * reference frame, e.g. at startup or when re-synchronising with an
     * external localisation source.
     */
    void reset();

    /**
     * @brief Sets the pose to an arbitrary value.
     *
     * Useful for re-synchronising odometry with an external reference
     * (e.g. GPS fix, marker detection, manual placement on a known spot).
     */
    void setPose(float x, float y, float thetaRadians);

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    float getX() const;
    float getY() const;

    /** @brief Heading in radians, normalized to [-pi, pi]. */
    float getTheta() const;

    /** @brief Distance travelled by the robot's centre on the last update() call, in meters. Signed: negative if moving backward. */
    float getLastLinearDisplacement() const;

    /** @brief Heading change on the last update() call, in radians. */
    float getLastAngularDisplacement() const;

    float getWheelBase() const;
    float getWheelRadius() const;

private:
    const float _wheelRadiusMeters;
    const float _wheelBaseMeters;

    float _x;
    float _y;
    float _theta; // radians, normalized to [-pi, pi]

    float _lastLinearDisplacement;
    float _lastAngularDisplacement;

    /** Wraps an angle (radians) into [-pi, pi]. */
    static float _normalizeAngle(float angleRadians);
};
