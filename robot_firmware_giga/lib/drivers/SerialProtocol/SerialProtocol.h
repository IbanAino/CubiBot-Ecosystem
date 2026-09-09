#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Custom ASCII serial protocol between the Arduino and the
 *        Raspberry Pi (running ROS 2).
 *
 * This class only knows about text formatting and parsing — it has no
 * knowledge of motors, encoders, PID controllers, or odometry math. It
 * is the single place where the wire format is defined, so the format
 * can evolve (e.g. switch to binary later) without touching any other
 * class in the project.
 *
 * Wire format (newline-terminated ASCII lines):
 *
 *   Arduino -> Raspberry Pi (telemetry, sent periodically):
 *     ODOM,<x>,<y>,<theta>,<leftMeasuredVel>,<rightMeasuredVel>,<leftStalled>,<rightStalled>\n
 *     - x, y in meters; theta in radians
 *     - leftMeasuredVel, rightMeasuredVel in rev/s (signed)
 *     - leftStalled, rightStalled as 0 or 1
 *
 *   Arduino -> Raspberry Pi (bumper event, sent only on state change):
 *     BUMPER,<left>,<right>\n
 *     - left, right as 0 (not pressed) or 1 (pressed)
 *     - sent once on press AND once on release (CHANGE trigger), so
 *       the Raspberry Pi / ROS 2 is informed of both collision and
 *       clearance events without having to poll
 *
 *   Raspberry Pi -> Arduino (velocity command, geometry_msgs/Twist style):
 *     CMD,<linearVelocity>,<angularVelocity>\n
 *     - linearVelocity in m/s, angularVelocity in rad/s
 *     - matches ROS 2's /cmd_vel convention (linear.x, angular.z) so the
 *       Raspberry Pi side can relay the topic with no conversion
 *
 *   Raspberry Pi -> Arduino (stall recovery acknowledgement):
 *     RESET\n
 *     - no parameters; tells the Arduino side it is safe to clear any
 *       latched stall state and resume accepting velocity commands
 *
 *   Raspberry Pi -> Arduino (full stop):
 *     STOP\n
 *     - no parameters; unlike CMD,0.0,0.0 (which only sets the target
 *       velocity to zero and leaves the PID loop running, which can
 *       cause audible motor buzz from residual measurement noise around
 *       zero), STOP calls VelocityMotorController::stop() on both
 *       wheels: motor output is cut AND the PID internal state (integral
 *       term, previous error) is reset.
 *
 *   Raspberry Pi -> Arduino (hard/emergency stop):
 *     BRAKE\n
 *     - no parameters; like STOP, but calls
 *       VelocityMotorController::brake() instead — actively brakes the
 *       motor (L298N electrodynamic brake) rather than letting it coast,
 *       for a faster, more predictable stopping distance.
 *
 * Usage (Arduino side):
 *   SerialProtocol protocol;
 *   ...
 *   // Sending telemetry
 *   protocol.sendOdometry(Serial, x, y, theta, leftVel, rightVel, leftStalled, rightStalled);
 *
 *   // Receiving a command
 *   while (Serial.available()) {
 *       if (protocol.feed(Serial.read())) {
 *           if (protocol.getLastMessageType() == SerialProtocol::MessageType::CMD) {
 *               float linear  = protocol.getCommandLinearVelocity();
 *               float angular = protocol.getCommandAngularVelocity();
 *           }
 *       }
 *   }
 */
class SerialProtocol
{
public:
    enum class MessageType { NONE, CMD, RESET, STOP, BRAKE, UNKNOWN };

    SerialProtocol();

    // ------------------------------------------------------------------
    // Sending (Arduino -> Raspberry Pi)
    // ------------------------------------------------------------------

    /**
     * @brief Formats and sends one BUMPER event line.
     *
     * Call this only when bumper state has changed (event-driven), not
     * periodically — the receiver relies on the absence of BUMPER frames
     * to infer "no change since last event".
     *
     * @param output        The stream to write to (typically Serial).
     * @param leftPressed   True if the left bumper is currently pressed.
     * @param rightPressed  True if the right bumper is currently pressed.
     */
    void sendBumperEvent(Stream& output,
                          bool leftPressed,
                          bool rightPressed) const;

    /**
     * @brief Formats and sends one ODOM telemetry line.
     *
     * @param output            The stream to write to (typically Serial).
     * @param x, y              Robot pose, in meters.
     * @param theta             Robot heading, in radians.
     * @param leftMeasuredVel   Left wheel measured velocity, rev/s (signed).
     * @param rightMeasuredVel  Right wheel measured velocity, rev/s (signed).
     * @param leftStalled       True if the left wheel's stall watchdog is active.
     * @param rightStalled      True if the right wheel's stall watchdog is active.
     */
    void sendOdometry(Stream& output,
                       float x, float y, float theta,
                       float leftMeasuredVel, float rightMeasuredVel,
                       bool leftStalled, bool rightStalled) const;

    // ------------------------------------------------------------------
    // Receiving (Raspberry Pi -> Arduino)
    // ------------------------------------------------------------------

    /**
     * @brief Feeds one incoming byte into the internal line buffer.
     *
     * Call this once per byte available on the serial port. Accumulates
     * characters until a newline is found, then parses the complete line.
     *
     * @param incomingByte  The next byte read from the serial port.
     * @return true if a complete, successfully parsed message is now
     *         available (check getLastMessageType() — CMD or RESET —
     *         and the relevant accessors for CMD). Returns false
     *         otherwise — including when a line was completed but
     *         failed to parse (malformed input is silently discarded;
     *         getLastMessageType() will report UNKNOWN in that case so
     *         the caller can choose to log it).
     */
    bool feed(char incomingByte);

    /** @brief Type of the last successfully terminated message. */
    MessageType getLastMessageType() const;

    /** @brief Valid after a CMD message: linear velocity, m/s. */
    float getCommandLinearVelocity() const;

    /** @brief Valid after a CMD message: angular velocity, rad/s. */
    float getCommandAngularVelocity() const;

private:
    static const uint8_t LINE_BUFFER_SIZE = 64;

    char    _lineBuffer[LINE_BUFFER_SIZE];
    uint8_t _lineLength;

    MessageType _lastMessageType;
    float       _commandLinearVelocity;
    float       _commandAngularVelocity;

    /** Parses _lineBuffer (null-terminated) and updates internal state. */
    void _parseLine();
};