#include "motor_control.h"
#include "config.h"
#include <Arduino.h>
#include <PID_v1.h>


// --- Encoder Definitions ---
Encoder encoderLeft1(2, 3);    // Left Motor 1 encoder channels
Encoder encoderLeft2(18, 19);  // Left Motor 2 encoder channels
Encoder encoderRight1(20, 21); // Right Motor 1 encoder channels
Encoder encoderRight2(22, 23); // Right Motor 2 encoder channels

// --- PID Variables ---
double leftInput, leftOutput, leftSetpoint;
double rightInput, rightOutput, rightSetpoint;

// PID tuning parameters (adjust based on testing)
double Kp = 2.0, Ki = 0.5, Kd = 0.1;
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

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
// --- PID-Controlled Movement ---
void moveForward(int distance, int basePWM) {
    long targetPulses = distance / distancePerPulse;
    
    encoderLeft1.write(0);
    encoderLeft2.write(0);
    encoderRight1.write(0);
    encoderRight2.write(0);
    
    leftSetpoint = rightSetpoint = targetPulses;
    
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    while (true) {
        long leftPulses = (abs(encoderLeft1.read()) + abs(encoderLeft2.read())) / 2;
        long rightPulses = (abs(encoderRight1.read()) + abs(encoderRight2.read())) / 2;

        leftInput = leftPulses;
        rightInput = rightPulses;

        leftPID.Compute();
        rightPID.Compute();

        moveLeftMotorsForward(basePWM + leftOutput);
        moveRightMotorsForward(basePWM + rightOutput);

        if (leftPulses >= targetPulses && rightPulses >= targetPulses) break;
        delay(5);
    }

    stopMotors();
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
// --- PID-Controlled Rotation ---
void rotateLeft(float angle, int basePWM) {
    long targetPulses = (3.1416 * axleLength * (angle / 360.0)) / distancePerPulse;

    encoderLeft1.write(0);
    encoderLeft2.write(0);
    encoderRight1.write(0);
    encoderRight2.write(0);

    leftSetpoint = rightSetpoint = targetPulses;

    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    while (true) {
        long leftPulses = (abs(encoderLeft1.read()) + abs(encoderLeft2.read())) / 2;
        long rightPulses = (abs(encoderRight1.read()) + abs(encoderRight2.read())) / 2;

        leftInput = leftPulses;
        rightInput = rightPulses;

        leftPID.Compute();
        rightPID.Compute();

        moveLeftMotorsBackward(basePWM + leftOutput);
        moveRightMotorsForward(basePWM + rightOutput);

        if (leftPulses >= targetPulses && rightPulses >= targetPulses) break;
        delay(5);
    }

    stopMotors();
}

void rotateRight(float angle, int basePWM) {
    long targetPulses = (3.1416 * axleLength * (angle / 360.0)) / distancePerPulse;

    encoderLeft1.write(0);
    encoderLeft2.write(0);
    encoderRight1.write(0);
    encoderRight2.write(0);

    leftSetpoint = rightSetpoint = targetPulses;

    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    while (true) {
        long leftPulses = (abs(encoderLeft1.read()) + abs(encoderLeft2.read())) / 2;
        long rightPulses = (abs(encoderRight1.read()) + abs(encoderRight2.read())) / 2;

        leftInput = leftPulses;
        rightInput = rightPulses;

        leftPID.Compute();
        rightPID.Compute();

        moveLeftMotorsForward(basePWM + leftOutput);
        moveRightMotorsBackward(basePWM + rightOutput);

        if (leftPulses >= targetPulses && rightPulses >= targetPulses) break;
        delay(5);
    }

    stopMotors();
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
