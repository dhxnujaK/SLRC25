#include "motor_control.h"
#include "config.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "gyroscope.h"

// ToF Sensor
// VL53L0X tof1;

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
    unsigned long startTime = millis();  // Timeout mechanism

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

        if ((leftPulses >= targetPulses && rightPulses >= targetPulses) || (millis() - startTime > 5000)) break;  // Timeout after 5 seconds
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
    unsigned long startTime = millis();  // Timeout mechanism

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

        if ((leftPulses >= targetPulses && rightPulses >= targetPulses) || (millis() - startTime > 5000)) break;  // Timeout after 5 seconds
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
    
    resetGyroAngle();

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

        float filteredAngle = getFilteredAngle();
        Serial.print("Filtered Gyro Angle: "); Serial.println(filteredAngle);

        if ((leftPulses >= targetPulses && rightPulses >= targetPulses) || fabs(filteredAngle) >= angle) {
            break;
        }

        delay(5);
    }

    stopMotors();
}

void moveForwardUntilObstacle() {
    if (!tof1.init()) {  // Ensure proper initialization of tof1
        Serial.println("ToF sensor initialization failed!");
        return;
    }

    // Reset encoders to track distance traveled
    encoderLeft1.write(0);
    encoderRight1.write(0);

    // Start moving forward
    digitalWrite(EnablePin, HIGH);  // Enable motors
    moveLeftMotorsForward(150);
    moveRightMotorsForward(150);

    while (true) {
        int distance = tof1.readRangeSingleMillimeters();
        Serial.print("Distance: ");
        Serial.println(distance);

        // Stop if an obstacle is detected within 20 cm
        if (distance < 200) {
            stopMotors();
            break;
        }

        delay(10);  // Small delay to avoid excessive sensor polling
    }

    // Calculate distance traveled using encoder counts
    long leftPulses = abs(encoderLeft1.read());
    long rightPulses = abs(encoderRight1.read());
    long averagePulses = (leftPulses + rightPulses) / 2;

    // Convert encoder pulses to distance
    float distanceTraveled = averagePulses * distancePerPulse;
    Serial.print("Moved Forward: ");
    Serial.print(distanceTraveled);
    Serial.println(" mm");

    // Optionally, move backward the same distance
    moveBackwardDistance(distanceTraveled);
}

void moveBackwardDistance(float distance) {
    long targetPulses = distance / distancePerPulse;
    
    long initialLeftEncoder = encoderLeft1.read();
    long initialRightEncoder = encoderRight1.read();

    moveBackward(150);

    while (true) {
        long leftPulses = abs(encoderLeft1.read() - initialLeftEncoder);
        long rightPulses = abs(encoderRight1.read() - initialRightEncoder);
        long avgPulses = (leftPulses + rightPulses) / 2;

        if (avgPulses >= targetPulses) {
            stopMotors();
            break;
        }
    }

    Serial.println("Returned to starting point!");
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
