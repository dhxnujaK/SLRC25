#include "motor_control.h"
#include "config.h"
#include <Arduino.h>

// --- Encoder Definitions ---
Encoder encoderLeft1(2, 3);    // Left Motor 1 encoder channels
Encoder encoderLeft2(18, 19);  // Left Motor 2 encoder channels
Encoder encoderRight1(20, 21); // Right Motor 1 encoder channels
Encoder encoderRight2(22, 23); // Right Motor 2 encoder channels

// --- Individual Motor Control Functions ---
void moveLeftMotorsForward(int pwmVal) {
    digitalWrite(leftMotorDirPin1, HIGH);
    digitalWrite(leftMotorDirPin2, LOW);
    analogWrite(leftMotorPWMPin, pwmVal);
}

void moveLeftMotorsBackward(int pwmVal) {
    digitalWrite(leftMotorDirPin1, LOW);
    digitalWrite(leftMotorDirPin2, HIGH);
    analogWrite(leftMotorPWMPin, pwmVal);
}

void moveRightMotorsForward(int pwmVal) {
    digitalWrite(rightMotorDirPin1, HIGH);
    digitalWrite(rightMotorDirPin2, LOW);
    analogWrite(rightMotorPWMPin, pwmVal);
}

void moveRightMotorsBackward(int pwmVal) {
    digitalWrite(rightMotorDirPin1, LOW);
    digitalWrite(rightMotorDirPin2, HIGH);
    analogWrite(rightMotorPWMPin, pwmVal);
}

// --- Combined Movement Functions ---
void moveForward(int pwmVal) {
    moveLeftMotorsForward(pwmVal);
    moveRightMotorsForward(pwmVal);
}

void moveBackward(int pwmVal) {
    moveLeftMotorsBackward(pwmVal);
    moveRightMotorsBackward(pwmVal);
}

void moveLeftForward(int pwmVal) {
    moveLeftMotorsForward(pwmVal);
}

void moveRightForward(int pwmVal) {
    moveRightMotorsForward(pwmVal);
}

// --- Rotation Functions ---
void rotateLeft(float angle, int pwmVal) {
    // Reset encoder values
    encoderLeft1.write(0);
    encoderLeft2.write(0);
    encoderRight1.write(0);
    encoderRight2.write(0);

    // Calculate the required wheel travel distance for a given angle
    float distancePerWheel = 3.1416 * axleLength * (angle / 360.0);
    long targetPulses = distancePerWheel / distancePerPulse;

    Serial.print("Rotating Left: ");
    Serial.print(angle);
    Serial.print("° (Target pulses per wheel: ");
    Serial.print(targetPulses);
    Serial.println(")");

    // Start rotation
    moveLeftMotorsBackward(pwmVal);
    moveRightMotorsForward(pwmVal);

    // Wait until target pulses are reached
    while (true) {
        long lCount1 = abs(encoderLeft1.read());
        long lCount2 = abs(encoderLeft2.read());
        long rCount1 = abs(encoderRight1.read());
        long rCount2 = abs(encoderRight2.read());

        long leftAvg = (lCount1 + lCount2) / 2;
        long rightAvg = (rCount1 + rCount2) / 2;
        long avgCount = (leftAvg + rightAvg) / 2;

        if (avgCount >= targetPulses) break;
        delay(5);
    }

    stopMotors();
    Serial.println("Left rotation complete.");
}

void rotateRight(float angle, int pwmVal) {
    // Reset encoder values
    encoderLeft1.write(0);
    encoderLeft2.write(0);
    encoderRight1.write(0);
    encoderRight2.write(0);

    // Calculate the required wheel travel distance for a given angle
    float distancePerWheel = 3.1416 * axleLength * (angle / 360.0);
    long targetPulses = distancePerWheel / distancePerPulse;

    Serial.print("Rotating Right: ");
    Serial.print(angle);
    Serial.print("° (Target pulses per wheel: ");
    Serial.print(targetPulses);
    Serial.println(")");

    // Start rotation
    moveLeftMotorsForward(pwmVal);
    moveRightMotorsBackward(pwmVal);

    // Wait until target pulses are reached
    while (true) {
        long lCount1 = abs(encoderLeft1.read());
        long lCount2 = abs(encoderLeft2.read());
        long rCount1 = abs(encoderRight1.read());
        long rCount2 = abs(encoderRight2.read());

        long leftAvg = (lCount1 + lCount2) / 2;
        long rightAvg = (rCount1 + rCount2) / 2;
        long avgCount = (leftAvg + rightAvg) / 2;

        if (avgCount >= targetPulses) break;
        delay(5);
    }

    stopMotors();
    Serial.println("Right rotation complete.");
}

// --- Stop Motors Function ---
void stopMotors() {
    digitalWrite(leftMotorDirPin1, LOW);
    digitalWrite(leftMotorDirPin2, LOW);
    digitalWrite(rightMotorDirPin1, LOW);
    digitalWrite(rightMotorDirPin2, LOW);
    analogWrite(leftMotorPWMPin, 0);
    analogWrite(rightMotorPWMPin, 0);
}
