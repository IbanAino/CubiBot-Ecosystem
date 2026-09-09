#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Driver for a quadrature rotary incremental encoder.
 *
 * Supports an arbitrary number of instances (limited only by available
 * hardware interrupt pins). Each instance owns its own pins, its own
 * tick counters and its own speed-measurement timestamp — there is no
 * shared mutable state between instances.
 *
 * Usage:
 *   RotaryIncrementalEncoder leftWheel(2, 3, 11);   // pinA, pinB, ticksPerRev
 *   RotaryIncrementalEncoder rightWheel(18, 19, 11);
 *
 *   leftWheel.attachInterrupts(isrLeftA, isrLeftB);
 *   rightWheel.attachInterrupts(isrRightA, isrRightB);
 *
 * Because attachInterrupt() requires a plain function pointer (not a
 * method), you must declare one thin ISR wrapper per encoder instance
 * in your .ino / main .cpp:
 *
 *   void isrLeftA()  { leftWheel.handleChannelA(); }
 *   void isrLeftB()  { leftWheel.handleChannelB(); }
 *   void isrRightA() { rightWheel.handleChannelA(); }
 *   void isrRightB() { rightWheel.handleChannelB(); }
 */
class RotaryIncrementalEncoder
{
public:
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /**
     * @param pinA           Hardware interrupt pin for channel A.
     * @param pinB           Hardware interrupt pin for channel B.
     * @param ticksPerRev    Number of encoder ticks per full motor shaft revolution.
     */
    RotaryIncrementalEncoder(uint8_t pinA, uint8_t pinB, uint16_t ticksPerRev);

    /**
     * @brief Attach hardware interrupts to the ISR wrappers you defined.
     *
     * Call this once in setup(), after declaring your wrapper functions.
     * Both channels are triggered on CHANGE so direction is detected
     * on every edge (×4 resolution).
     *
     * @param isrA  Plain function pointer for channel A ISR wrapper.
     * @param isrB  Plain function pointer for channel B ISR wrapper.
     */
    void attachInterrupts(void (*isrA)(), void (*isrB)());

    // ------------------------------------------------------------------
    // Tick counter (absolute, cumulative, never reset automatically)
    // ------------------------------------------------------------------

    /**
     * @brief Returns the cumulative signed tick count since power-on (or last
     *        manual reset). Positive = forward, negative = reverse.
     *
     * This mirrors the ROS 2 convention: the consumer computes deltas
     * between two successive calls rather than relying on the driver to
     * reset the counter.
     *
     * Thread-safety: reading a 32-bit variable is not atomic on AVR.
     * Interrupts are briefly disabled to take a consistent snapshot.
     */
    int32_t getTicks() const;

	/** @brief Returns the configured ticks-per-revolution value. */
	uint16_t getTicksPerRev() const { return _ticksPerRev; }

    /**
     * @brief Resets the cumulative tick counter to zero.
     *
     * Use sparingly — prefer delta computation between two getTicks()
     * calls. Useful at initialisation or after a known reference point.
     */
    void resetTicks();

    // ------------------------------------------------------------------
    // Speed
    // ------------------------------------------------------------------

    /**
     * @brief Returns the instantaneous motor speed in revolutions per second.
     *
     * Speed is estimated over the elapsed time since the previous call to
     * getSpeed() (or since the encoder was constructed if never called).
     * The tick delta over that window is divided by the elapsed time and
     * by ticksPerRev.
     *
     * Calling this function resets the internal speed-measurement window.
     * The returned value is always positive (magnitude only); use getTicks()
     * to determine direction.
     */
    float getSpeed();

    // ------------------------------------------------------------------
    // ISR helpers — call these from your wrapper functions, not directly
    // ------------------------------------------------------------------

    /** Called from the channel-A ISR wrapper. */
    void handleChannelA();

    /** Called from the channel-B ISR wrapper. */
    void handleChannelB();

private:
    // Pins
    const uint8_t _pinA;
    const uint8_t _pinB;
    const uint16_t _ticksPerRev;

    // Tick counter — modified in ISR, must be volatile
    volatile int32_t _ticks;

    // Speed measurement — timestamp and tick snapshot at last getSpeed() call
    uint32_t _speedWindowStartMs;
    int32_t  _speedWindowStartTicks;
};
