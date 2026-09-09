#include "BumperSensor.h"

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

BumperSensor::BumperSensor(uint8_t pin, uint16_t debounceDelayMs)
    : _pin(pin)
    , _debounceDelayMs(debounceDelayMs)
    , _pressed(false)
    , _hasNewEvent(false)
    , _lastChangeMs(0)
{
}

// -----------------------------------------------------------------------
// Interrupt attachment
// -----------------------------------------------------------------------

void BumperSensor::attachInterrupt(void (*isr)())
{
    // INPUT_PULLUP: pin reads HIGH at rest (switch open), LOW when
    // pressed (switch closes to GND) — active-low logic.
    pinMode(_pin, INPUT_PULLUP);

    // CHANGE: triggers on both press and release, so the firmware can
    // notify ROS 2 of both collision and clearance events.
    ::attachInterrupt(digitalPinToInterrupt(_pin), isr, CHANGE);

    // Read initial state so isPressed() is valid before any interrupt fires.
    _pressed = (digitalRead(_pin) == LOW);
}

// -----------------------------------------------------------------------
// ISR helper
// -----------------------------------------------------------------------

void BumperSensor::handleInterrupt()
{
    const uint32_t nowMs = millis();

    // Software debounce: ignore transitions that occur too soon after
    // the previous accepted one (mechanical bounce, or noise that
    // survived the hardware RC filter).
    if ((nowMs - _lastChangeMs) < _debounceDelayMs) {
        return;
    }

    // Active-low: LOW means the switch is closed (bumper pressed).
    const bool currentlyPressed = (digitalRead(_pin) == LOW);

    // Only accept a change if the state actually differs — guards against
    // a spurious re-trigger on the same level.
    if (currentlyPressed != _pressed) {
        _pressed      = currentlyPressed;
        _hasNewEvent  = true;
        _lastChangeMs = nowMs;
    }
}

// -----------------------------------------------------------------------
// Accessors
// -----------------------------------------------------------------------

bool BumperSensor::isPressed() const
{
    // noInterrupts/interrupts snapshot: _pressed is a single bool, which
    // is an atomic read on AVR — no protection needed here. Kept for
    // clarity and in case of future porting to non-atomic platforms.
    return _pressed;
}

bool BumperSensor::hasNewEvent() const
{
    return _hasNewEvent;
}

void BumperSensor::clearEvent()
{
    // Called from the main loop — safe to write without noInterrupts()
    // since the ISR only ever sets this flag to true, never reads it
    // to make a branching decision that would be corrupted by a race.
    _hasNewEvent = false;
}