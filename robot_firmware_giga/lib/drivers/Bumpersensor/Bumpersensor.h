#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Driver for a single bumper switch connected to a hardware
 *        interrupt pin, with software debounce complementing the
 *        recommended hardware debounce (100nF capacitor between pin
 *        and GND).
 *
 * Hardware wiring:
 *   - Switch between the interrupt pin and GND
 *   - Internal pull-up resistor enabled (INPUT_PULLUP): pin reads HIGH
 *     at rest, LOW when the bumper is pressed (active-low logic)
 *   - Optional: 100nF capacitor in parallel with the switch (between
 *     pin and GND) for hardware debounce — eliminates most bounce
 *     before the ISR is even invoked, making the software debounce
 *     a lightweight safety net rather than the primary filter
 *
 * Software debounce:
 *   The ISR records the timestamp of each transition. A state change
 *   is only accepted if at least DEBOUNCE_DELAY_MS milliseconds have
 *   elapsed since the previous accepted change. This ignores spurious
 *   re-triggers from mechanical bounce that the hardware filter may
 *   not have fully suppressed.
 *
 * Usage:
 *   BumperSensor leftBumper(18, 50);
 *   BumperSensor rightBumper(19, 50);
 *
 *   void isrLeftBumper()  { leftBumper.handleInterrupt(); }
 *   void isrRightBumper() { rightBumper.handleInterrupt(); }
 *
 *   leftBumper.attachInterrupt(isrLeftBumper);
 *   rightBumper.attachInterrupt(isrRightBumper);
 *
 *   if (leftBumper.hasNewEvent()) {
 *       bool pressed = leftBumper.isPressed();
 *       leftBumper.clearEvent();
 *   }
 */
class BumperSensor
{
public:
    /**
     * @param pin              Hardware interrupt pin (2, 3, 18, 19, 20,
     *                         or 21 on Arduino Mega).
     * @param debounceDelayMs  Minimum time between two accepted state
     *                         changes, in milliseconds. Typical values:
     *                         20–50ms for mechanical switches.
     */
    BumperSensor(uint8_t pin, uint16_t debounceDelayMs = 50);

    /**
     * @brief Attaches the hardware interrupt to the provided ISR wrapper.
     *
     * Call once in setup(), after declaring your wrapper function:
     *   void isrLeft() { leftBumper.handleInterrupt(); }
     *   leftBumper.attachInterrupt(isrLeft);
     *
     * Triggered on CHANGE (both press and release) so the firmware can
     * distinguish a collision event from a release.
     */
    void attachInterrupt(void (*isr)());

    /**
     * @brief ISR helper — call this from your wrapper function.
     *
     * Reads the pin state and applies the software debounce filter.
     * Keeps execution minimal (no Serial, no malloc, no blocking calls)
     * as required for interrupt service routines.
     */
    void handleInterrupt();

    // ------------------------------------------------------------------
    // State accessors (called from the main loop, not from an ISR)
    // ------------------------------------------------------------------

    /** @brief Returns true if the bumper is currently pressed. */
    bool isPressed() const;

    /**
     * @brief Returns true if a new (debounced) state change has occurred
     *        since the last clearEvent() call.
     *
     * Use this to detect both press (collision) and release events, so
     * ROS 2 can be informed when the bumper is cleared as well as when
     * it is triggered.
     */
    bool hasNewEvent() const;

    /**
     * @brief Clears the pending event flag.
     *
     * Call this after processing a hasNewEvent() == true condition to
     * acknowledge the event and re-arm detection for the next change.
     */
    void clearEvent();

private:
    const uint8_t  _pin;
    const uint16_t _debounceDelayMs;

    // Modified in ISR — must be volatile
    volatile bool     _pressed;
    volatile bool     _hasNewEvent;
    volatile uint32_t _lastChangeMs;
};