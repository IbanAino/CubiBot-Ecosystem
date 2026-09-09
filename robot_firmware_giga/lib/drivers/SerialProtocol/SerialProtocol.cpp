#include "SerialProtocol.h"
#include <stdlib.h> // atof
#include <string.h> // strncmp, strchr

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

SerialProtocol::SerialProtocol()
    : _lineLength(0)
    , _lastMessageType(MessageType::NONE)
    , _commandLinearVelocity(0.0f)
    , _commandAngularVelocity(0.0f)
{
    _lineBuffer[0] = '\0';
}

// -----------------------------------------------------------------------
// Sending
// -----------------------------------------------------------------------

void SerialProtocol::sendBumperEvent(Stream& output,
                                       bool leftPressed,
                                       bool rightPressed) const
{
    output.print("BUMPER,");
    output.print(leftPressed  ? 1 : 0);
    output.print(',');
    output.println(rightPressed ? 1 : 0);
}

void SerialProtocol::sendOdometry(Stream& output,
                                    float x, float y, float theta,
                                    float leftMeasuredVel, float rightMeasuredVel,
                                    bool leftStalled, bool rightStalled) const
{
    output.print("ODOM,");
    output.print(x, 4);
    output.print(',');
    output.print(y, 4);
    output.print(',');
    output.print(theta, 4);
    output.print(',');
    output.print(leftMeasuredVel, 4);
    output.print(',');
    output.print(rightMeasuredVel, 4);
    output.print(',');
    output.print(leftStalled ? 1 : 0);
    output.print(',');
    output.println(rightStalled ? 1 : 0); // println: sends the trailing '\n'
}

// -----------------------------------------------------------------------
// Receiving
// -----------------------------------------------------------------------

bool SerialProtocol::feed(char incomingByte)
{
    // Line terminator: parse what we have accumulated so far.
    if (incomingByte == '\n') {
        _lineBuffer[_lineLength] = '\0';
        _parseLine();
        _lineLength = 0; // reset buffer for the next line
        return (_lastMessageType != MessageType::UNKNOWN
                && _lastMessageType != MessageType::NONE);
    }

    // Ignore carriage return (common with \r\n line endings).
    if (incomingByte == '\r') {
        return false;
    }

    // Guard against buffer overflow: a malformed/oversized line is
    // discarded rather than allowed to overwrite adjacent memory.
    if (_lineLength >= (LINE_BUFFER_SIZE - 1)) {
        _lineLength = 0; // drop the line, start fresh
        _lastMessageType = MessageType::UNKNOWN;
        return false;
    }

    _lineBuffer[_lineLength++] = incomingByte;
    return false;
}

SerialProtocol::MessageType SerialProtocol::getLastMessageType() const
{
    return _lastMessageType;
}

float SerialProtocol::getCommandLinearVelocity() const
{
    return _commandLinearVelocity;
}

float SerialProtocol::getCommandAngularVelocity() const
{
    return _commandAngularVelocity;
}

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

void SerialProtocol::_parseLine()
{
    // RESET message: no parameters, exact match required.
    if (strcmp(_lineBuffer, "RESET") == 0) {
        _lastMessageType = MessageType::RESET;
        return;
    }

    // STOP message: no parameters, exact match required.
    if (strcmp(_lineBuffer, "STOP") == 0) {
        _lastMessageType = MessageType::STOP;
        return;
    }

    // BRAKE message: no parameters, exact match required.
    if (strcmp(_lineBuffer, "BRAKE") == 0) {
        _lastMessageType = MessageType::BRAKE;
        return;
    }

    // Expected format: CMD,<linear>,<angular>
    //
    // AVR-libc does not provide strtof() (only a subset of the standard
    // C library is available on 8-bit AVR targets), so we cannot rely on
    // the "pointer to first unparsed character" trick to detect parse
    // failures. Instead we manually locate the comma separators and
    // convert each isolated segment with atof(), which AVR-libc does
    // provide. The trade-off: a malformed numeric segment (e.g. empty
    // string) silently converts to 0.0 rather than being detected as an
    // error — acceptable here since the leading "CMD," and comma-count
    // checks already reject most malformed lines.

    if (strncmp(_lineBuffer, "CMD,", 4) != 0) {
        _lastMessageType = MessageType::UNKNOWN;
        return;
    }

    char* firstValue = _lineBuffer + 4; // skip "CMD,"

    // Find the comma separating the two numeric fields.
    char* comma = strchr(firstValue, ',');
    if (comma == nullptr) {
        _lastMessageType = MessageType::UNKNOWN;
        return;
    }

    // Temporarily split the buffer in place: replace the comma with a
    // null terminator so atof() on firstValue stops exactly there.
    *comma = '\0';
    char* secondValue = comma + 1;

    if (*firstValue == '\0' || *secondValue == '\0') {
        // Empty field on either side of the comma — reject rather than
        // silently treat as 0.0.
        _lastMessageType = MessageType::UNKNOWN;
        return;
    }

    _commandLinearVelocity  = static_cast<float>(atof(firstValue));
    _commandAngularVelocity = static_cast<float>(atof(secondValue));
    _lastMessageType        = MessageType::CMD;
}