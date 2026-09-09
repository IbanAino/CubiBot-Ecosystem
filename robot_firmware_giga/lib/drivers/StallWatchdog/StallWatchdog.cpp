#include "StallWatchdog.h"
#include <math.h>

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------

StallWatchdog::StallWatchdog(float minExpectedMagnitude, uint32_t stallTimeoutMs)
    : _minExpectedMagnitude(fabsf(minExpectedMagnitude))
    , _stallTimeoutMs(stallTimeoutMs)
    , _belowThresholdDurationMs(0)
    , _isStalled(false)
{
}

// -----------------------------------------------------------------------
// Core update
// -----------------------------------------------------------------------

void StallWatchdog::update(float command, float measurement, float dtSeconds)
{
    // Already latched as stalled: stay stalled until an explicit reset().
    // See header rationale — we don't want silent self-recovery.
    if (_isStalled) {
        return;
    }

    const bool commandIsActive = fabsf(command) >= _minExpectedMagnitude;
    const bool measurementIsResponding = fabsf(measurement) >= _minExpectedMagnitude;

    if (!commandIsActive || measurementIsResponding) {
        // Either nothing is being commanded, or the actuator is responding
        // normally: no stall risk right now, reset the countdown.
        _belowThresholdDurationMs = 0;
        return;
    }

    // Command is active but measurement is not responding: accumulate time.
    _belowThresholdDurationMs += static_cast<uint32_t>(dtSeconds * 1000.0f);

    if (_belowThresholdDurationMs >= _stallTimeoutMs) {
        _isStalled = true;
    }
}

bool StallWatchdog::isStalled() const
{
    return _isStalled;
}

void StallWatchdog::reset()
{
    _isStalled = false;
    _belowThresholdDurationMs = 0;
}
