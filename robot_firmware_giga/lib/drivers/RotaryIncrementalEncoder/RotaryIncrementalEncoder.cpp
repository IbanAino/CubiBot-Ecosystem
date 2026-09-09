#include "RotaryIncrementalEncoder.h"

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

RotaryIncrementalEncoder::RotaryIncrementalEncoder(uint8_t pinA,
                                                   uint8_t pinB,
                                                   uint16_t ticksPerRev)
    : _pinA(pinA)
    , _pinB(pinB)
    , _ticksPerRev(ticksPerRev)
    , _ticks(0)
    , _speedWindowStartMs(0)
    , _speedWindowStartTicks(0)
{
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
}

// -----------------------------------------------------------------------
// Interrupt attachment
// -----------------------------------------------------------------------

void RotaryIncrementalEncoder::attachInterrupts(void (*isrA)(), void (*isrB)())
{
    // Trigger on CHANGE (both edges) for ×4 quadrature resolution
    attachInterrupt(digitalPinToInterrupt(_pinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), isrB, CHANGE);

    // Snapshot the start time so getSpeed() works from the very first call
    _speedWindowStartMs    = millis();
    _speedWindowStartTicks = 0;
}

// -----------------------------------------------------------------------
// Tick counter
// -----------------------------------------------------------------------

int32_t RotaryIncrementalEncoder::getTicks() const
{
    // On AVR, 32-bit reads are not atomic: disable interrupts for a
    // consistent snapshot, then re-enable immediately.
    noInterrupts();
    const int32_t snapshot = _ticks;
    interrupts();
    return snapshot;
}

void RotaryIncrementalEncoder::resetTicks()
{
    noInterrupts();
    _ticks = 0;
    interrupts();

    // Also reset the speed window so the next getSpeed() call is coherent
    _speedWindowStartMs    = millis();
    _speedWindowStartTicks = 0;
}

// -----------------------------------------------------------------------
// Speed
// -----------------------------------------------------------------------

float RotaryIncrementalEncoder::getSpeed()
{
    // Atomic snapshot
    noInterrupts();
    const int32_t currentTicks = _ticks;
    interrupts();

    const uint32_t currentMs = millis();

    const int32_t  deltaTicks = currentTicks - _speedWindowStartTicks;
    const uint32_t deltaMs    = currentMs    - _speedWindowStartMs;

    // Slide the window forward
    _speedWindowStartTicks = currentTicks;
    _speedWindowStartMs    = currentMs;

    if (deltaMs == 0) {
        return 0.0f; // Avoid division by zero on the very first call
    }

    // |deltaTicks| / ticksPerRev  →  revolutions
    // divided by deltaMs/1000     →  per second
    const float revolutions = static_cast<float>(abs(deltaTicks))
                              / static_cast<float>(_ticksPerRev);
    const float seconds     = static_cast<float>(deltaMs) / 1000.0f;

    return revolutions / seconds;
}

// -----------------------------------------------------------------------
// ISR helpers — quadrature decoding
// -----------------------------------------------------------------------

/**
 * Quadrature truth table (×4 decoding, both channels, both edges):
 *
 *   A  B  | direction
 *  -------|-----------
 *   H  L  |   +1
 *   H  H  |   -1
 *   L  H  |   +1
 *   L  L  |   -1
 *
 * Reading digitalRead() inside an ISR is safe on AVR but slow.
 * For higher encoder frequencies consider reading the port register directly:
 *   bool stateA = (PINE & (1 << PE4)) != 0;  // example for pin 2 on Mega
 */

void RotaryIncrementalEncoder::handleChannelA()
{
    const bool stateA = digitalRead(_pinA);
    const bool stateB = digitalRead(_pinB);

    // Forward when A and B differ, reverse when they match
    if (stateA != stateB) {
        _ticks++;
    } else {
        _ticks--;
    }
}

void RotaryIncrementalEncoder::handleChannelB()
{
    const bool stateA = digitalRead(_pinA);
    const bool stateB = digitalRead(_pinB);

    // Opposite logic from channel A
    if (stateA == stateB) {
        _ticks++;
    } else {
        _ticks--;
    }
}
