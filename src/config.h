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
// Left Motor 1:
const int leftMotor1PWMPin   = 5;  // PWM control
const int leftMotor1DirPin1  = 7;  // Direction pin 1
const int leftMotor1DirPin2  = 8;  // Direction pin 2

// Left Motor 2:
const int leftMotor2PWMPin   = 6;  // PWM control
const int leftMotor2DirPin1  = 9;  // Direction pin 1
const int leftMotor2DirPin2  = 10; // Direction pin 2

// Right Motor 1:
const int rightMotor1PWMPin  = 11;  // PWM control
const int rightMotor1DirPin1 = 12;  // Direction pin 1
const int rightMotor1DirPin2 = 13;  // Direction pin 2

// Right Motor 2:
const int rightMotor2PWMPin  = 14;  // PWM control
const int rightMotor2DirPin1 = 15;  // Direction pin 1
const int rightMotor2DirPin2 = 16;  // Direction pin 2

// --- Odometry and Geometry Parameters ---
const float wheelDiameter = 0.065;    // meters (e.g., 65 mm)
const int pulsesPerRevolution = 20;     // update according to your encoder specs
const float gearRatio = 1.0;            // adjust if needed
const float wheelCircumference = 3.1416 * wheelDiameter;
const float distancePerPulse = wheelCircumference / (pulsesPerRevolution * gearRatio);

// For a differential-drive robot:
const float axleLength = 0.15;  // meters (distance between left and right wheels)
const int EnablePin = 17;       // Enable pin for TB6612FNG motor driver

// TOF
#define TOF1_XSHUT  18
extern VL53L0X tof1;

#endif