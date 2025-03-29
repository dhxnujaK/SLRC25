#ifndef CONFIG_H
#define CONFIG_H

#include <Encoder.h>
#include <VL53L0X.h>

// --- Encoder Setup ---
// Left motors
extern Encoder encoderLeft1;    // Left Motor 1 encoder
extern Encoder encoderLeft2;    // Left Motor 2 encoder

// Right motors
extern Encoder encoderRight1;   // Right Motor 1 encoder
extern Encoder encoderRight2;   // Right Motor 2 encoder

// --- Motor Driver Pins (TB6612FNG) ---
// Left motors:
const int leftMotorPWMPin   = 5;  // PWM control
const int leftMotorDirPin1  = 7;  // Direction pin 1
const int leftMotorDirPin2  = 8;  // Direction pin 2

// Right motors:
const int rightMotorPWMPin  = 6;  // PWM control
const int rightMotorDirPin1 = 9;  // Direction pin 1
const int rightMotorDirPin2 = 10; // Direction pin 2

// --- Odometry and Geometry Parameters ---
const float wheelDiameter = 0.065;    // meters (e.g., 65 mm)
const int pulsesPerRevolution = 20;     // update according to your encoder specs
const float gearRatio = 1.0;            // adjust if needed
const float wheelCircumference = 3.1416 * wheelDiameter;
const float distancePerPulse = wheelCircumference / (pulsesPerRevolution * gearRatio);

// For a differential-drive robot:
const float axleLength = 0.15;  // meters (distance between left and right wheels)
const int EnablePin = 13; // Enable pin for TB6612FNG motor driver

// TOF
#define TOF1_XSHUT  7
extern VL53L0X tof1;


#endif