#pragma once
#include <Stream.h>
#include "SerialProtocol.h"

/**
 * @brief Communication channel between the Arduino firmware and the
 *        Raspberry Pi / ROS 2.
 *
 * Responsibilities:
 *   - Owns the reference to the physical Stream (Serial port)
 *   - Orchestrates reception: polls bytes from the stream and feeds
 *     them to SerialProtocol one at a time
 *   - Orchestrates sending: delegates frame formatting to SerialProtocol
 *     and passes the stream to it
 *
 * What it is NOT responsible for:
 *   - Frame format (that belongs to SerialProtocol)
 *   - Motor control or sensor logic (that belongs to main.cpp and the
 *     respective controller classes)
 *
 * Usage constraint:
 *   commandAvailable() returns true only until the next call to update().
 *   Always read and dispatch the command in the same loop() iteration
 *   as the commandAvailable() check, before update() is called again.
 */
class CommInterface {
public:
    // Dependency injection: any Stream and the shared protocol parser
    CommInterface(Stream& port, SerialProtocol& protocol);

    /**
     * @brief Call once per loop() iteration — non-blocking.
     *
     * Reads all bytes currently available on the stream and feeds them
     * to protocol.feed(). Sets the internal commandReady flag if a
     * complete, valid frame was received during this call.
     *
     * Resets commandReady at the start of each call — the caller must
     * check commandAvailable() and dispatch the command before the next
     * update() call.
     */
    void update();

    /**
     * @brief Returns true if a complete command frame was received
     *        during the most recent update() call.
     *
     * Valid only until the next update() call.
     */
    bool commandAvailable() const;

    // ------------------------------------------------------------------
    // Sending — high-level (one method per outgoing frame type)
    // ------------------------------------------------------------------

    void sendOdometry(float x, float y, float theta,
                      float leftVel, float rightVel,
                      bool leftStalled, bool rightStalled);

    /**
     * @brief Sends a BUMPER event frame.
     *
     * Call only on bumper state change (event-driven), not periodically.
     * Sent on both press AND release so ROS 2 knows when the bumper
     * is cleared as well as when a collision occurs.
     */
    void sendBumperEvent(bool leftPressed, bool rightPressed);

    // ------------------------------------------------------------------
    // Sending — low-level (debug only, do NOT use in production)
    // ------------------------------------------------------------------

    /**
     * @brief Sends raw bytes directly on the stream, bypassing
     *        SerialProtocol entirely.
     *
     * WARNING: for debug/inspection only. Sending malformed data can
     * corrupt the ASCII framing protocol and confuse the receiver.
     * Remove all calls to this method before deploying.
     */
    void sendRawDebug(const uint8_t* data, uint8_t len);

private:
    Stream&         _port;
    SerialProtocol& _protocol;
    bool            _commandReady = false;
};