#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Driver for a single DC motor controlled via an L298N H-bridge.
 *
 * Each L298N channel uses three pins:
 *   - pinEnable : PWM pin connected to ENA or ENB  (speed control)
 *   - pinIN1    : direction pin 1 (IN1 or IN3)
 *   - pinIN2    : direction pin 2 (IN2 or IN4)
 *
 * Usage:
 *   DCMotor leftMotor(9, 4, 5);    // ENA, IN1, IN2
 *   DCMotor rightMotor(10, 6, 7);  // ENB, IN3, IN4
 *
 *   leftMotor.setSpeed(180);   // 0–255
 *   leftMotor.forward();
 *   delay(2000);
 *   leftMotor.stop();
 */
class DCMotor
{
public:
    // ------------------------------------------------------------------
    // Types
    // ------------------------------------------------------------------

    enum class Direction { FORWARD, BACKWARD, STOPPED };

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /**
     * @param pinEnable  PWM-capable pin connected to ENA / ENB.
     * @param pinIN1     First direction pin (IN1 or IN3).
     * @param pinIN2     Second direction pin (IN2 or IN4).
     */
    DCMotor(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2);

    // ------------------------------------------------------------------
    // Speed  (0 = stopped, 255 = full speed)
    // ------------------------------------------------------------------

    /**
     * @brief Sets the motor speed without changing direction.
     * @param speed  PWM duty cycle in [0, 255].
     *               Passing 0 is equivalent to calling stop().
     */
    void setSpeed(uint8_t speed);

    /** @brief Returns the last speed value set (0–255). */
    uint8_t getSpeed() const;

    // ------------------------------------------------------------------
    // Direction
    // ------------------------------------------------------------------

    /** @brief Runs the motor forward at the current speed. */
    void forward();

    /** @brief Runs the motor backward at the current speed. */
    void backward();

    /**
     * @brief Coasts to a stop (both IN pins LOW, PWM still active).
     *
     * The L298N will let the motor spin down freely.
     * Use brake() if you need active braking.
     */
    void stop();

    /**
     * @brief Active braking (both IN pins HIGH).
     *
     * The L298N short-circuits the motor windings, producing a fast
     * electrodynamic brake. More aggressive than stop().
     */
    void brake();

    /** @brief Returns the current direction / state. */
    Direction getDirection() const;

private:
    const uint8_t _pinEnable;
    const uint8_t _pinIN1;
    const uint8_t _pinIN2;

    uint8_t   _speed;
    Direction _direction;

    /** Internal helper: writes the three pins atomically. */
    void _apply(uint8_t in1, uint8_t in2) const;
};
