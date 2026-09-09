/**
 * Example: two wheel encoders on an Arduino Mega
 *
 * Left wheel  : channel A → pin 2, channel B → pin 3
 * Right wheel : channel A → pin 18, channel B → pin 19
 * Encoder resolution : 11 ticks per revolution
 */

#include "RotaryIncrementalEncoder.h"

static const uint16_t TICKS_PER_REV = 11;

RotaryIncrementalEncoder leftWheel(2, 3, TICKS_PER_REV);
RotaryIncrementalEncoder rightWheel(20, 21, TICKS_PER_REV);

// --- Thin ISR wrappers (one pair per instance, mandatory) ---------------
void isrLeftA()  { leftWheel.handleChannelA(); }
void isrLeftB()  { leftWheel.handleChannelB(); }
void isrRightA() { rightWheel.handleChannelA(); }
void isrRightB() { rightWheel.handleChannelB(); }
// ------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    leftWheel.attachInterrupts(isrLeftA, isrLeftB);
    rightWheel.attachInterrupts(isrRightA, isrRightB);
}

void loop()
{
    // --- Delta tick example (ROS 2 style) ---
    static int32_t prevLeftTicks  = 0;
    static int32_t prevRightTicks = 0;

    const int32_t leftTicks  = leftWheel.getTicks();
    const int32_t rightTicks = rightWheel.getTicks();

    const int32_t deltaLeft  = leftTicks  - prevLeftTicks;
    const int32_t deltaRight = rightTicks - prevRightTicks;

    prevLeftTicks  = leftTicks;
    prevRightTicks = rightTicks;

    // --- Speed ---
    const float leftSpeed  = leftWheel.getSpeed();   // rev/s
    const float rightSpeed = rightWheel.getSpeed();  // rev/s

    Serial.print("L ticks: "); Serial.print(leftTicks);
    Serial.print("  dL: ");    Serial.print(deltaLeft);
    Serial.print("  spd: ");   Serial.print(leftSpeed, 2);
    Serial.print(" rev/s  |  ");
    Serial.print("R ticks: "); Serial.print(rightTicks);
    Serial.print("  dR: ");    Serial.print(deltaRight);
    Serial.print("  spd: ");   Serial.println(rightSpeed, 2);

    delay(100);
}
