#include "motor_control.h"
#include "config.h"
#include <Arduino.h>

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
void rotateLeft(int pwmVal) {
    moveLeftMotorsBackward(pwmVal);
    moveRightMotorsForward(pwmVal);
}

void rotateRight(int pwmVal) {
    moveLeftMotorsForward(pwmVal);
    moveRightMotorsBackward(pwmVal);
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
