#include "CommInterface.h"

CommInterface::CommInterface(Stream& port, SerialProtocol& protocol)
    : _port(port), _protocol(protocol) {}

// ---------------------------------------------------------------------------
// Reception
// ---------------------------------------------------------------------------

void CommInterface::update() {
    _commandReady = false;
    while (_port.available()) {
        const char b = static_cast<char>(_port.read());
        if (_protocol.feed(b)) {
            _commandReady = true;
        }
    }
}

bool CommInterface::commandAvailable() const {
    return _commandReady;
}

// ---------------------------------------------------------------------------
// Sending — high-level
// ---------------------------------------------------------------------------

void CommInterface::sendOdometry(float x, float y, float theta,
                                  float leftVel, float rightVel,
                                  bool leftStalled, bool rightStalled)
{
    _protocol.sendOdometry(_port, x, y, theta, leftVel, rightVel,
                            leftStalled, rightStalled);
}

void CommInterface::sendBumperEvent(bool leftPressed, bool rightPressed)
{
    _protocol.sendBumperEvent(_port, leftPressed, rightPressed);
}

// ---------------------------------------------------------------------------
// Sending — low-level (debug only, do NOT use in production)
// ---------------------------------------------------------------------------

void CommInterface::sendRawDebug(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        _port.write(data[i]);
    }
}