/* #include "motor_control.h"
#include "config.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <VL53L0X.h>

// ToF Sensor
// VL53L0X tof1;

// --- Encoder Definitions ---
Encoder encoderLeft1(34, 35);    // Left Motor 1 encoder channels
Encoder encoderLeft2(37, 36);  // Left Motor 2 encoder channels
Encoder encoderRight1(26, 27); // Right Motor 1 encoder channels
Encoder encoderRight2(29, 28); // Right Motor 2 encoder channels

// Use external declarations for interrupt variables
extern volatile long left1Pos, left2Pos, right1Pos, right2Pos;

// --- PID Variables ---
double leftInput, leftOutput, leftSetpoint;
double rightInput, rightOutput, rightSetpoint;

// PID tuning parameters (adjust based on testing)
double Kp = 2.0, Ki = 0.5, Kd = 0.1;
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

// --- Individual Motor Control Functions ---
void moveLeftMotor1Forward(int pwmVal) {
    digitalWrite(leftMotor1DirPin1, HIGH);
    digitalWrite(leftMotor1DirPin2, LOW);
    analogWrite(leftMotor1PWMPin, pwmVal);
}

void moveLeftMotor1Backward(int pwmVal) {
    digitalWrite(leftMotor1DirPin1, LOW);
    digitalWrite(leftMotor1DirPin2, HIGH);
    analogWrite(leftMotor1PWMPin, pwmVal);
}

void moveLeftMotor2Forward(int pwmVal) {
    digitalWrite(leftMotor2DirPin1, HIGH);
    digitalWrite(leftMotor2DirPin2, LOW);
    analogWrite(leftMotor2PWMPin, pwmVal);
}

void moveLeftMotor2Backward(int pwmVal) {
    digitalWrite(leftMotor2DirPin1, LOW);
    digitalWrite(leftMotor2DirPin2, HIGH);
    analogWrite(leftMotor2PWMPin, pwmVal);
}

void moveRightMotor1Forward(int pwmVal) {
    digitalWrite(rightMotor1DirPin1, HIGH);
    digitalWrite(rightMotor1DirPin2, LOW);
    analogWrite(rightMotor1PWMPin, pwmVal);
}

void moveRightMotor1Backward(int pwmVal) {
    digitalWrite(rightMotor1DirPin1, LOW);
    digitalWrite(rightMotor1DirPin2, HIGH);
    analogWrite(rightMotor1PWMPin, pwmVal);
}

void moveRightMotor2Forward(int pwmVal) {
    digitalWrite(rightMotor2DirPin1, HIGH);
    digitalWrite(rightMotor2DirPin2, LOW);
    analogWrite(rightMotor2PWMPin, pwmVal);
}

void moveRightMotor2Backward(int pwmVal) {
    digitalWrite(rightMotor2DirPin1, LOW);
    digitalWrite(rightMotor2DirPin2, HIGH);
    analogWrite(rightMotor2PWMPin, pwmVal);
}

// --- Combined Movement Functions ---
void moveLeftMotorsForward(int pwmVal) {
    Serial.println("Moving left motors forward");
    moveLeftMotor1Forward(pwmVal);
    moveLeftMotor2Forward(pwmVal);
}

void moveLeftMotorsBackward(int pwmVal) {
    moveLeftMotor1Backward(pwmVal);
    moveLeftMotor2Backward(pwmVal);
}

void moveRightMotorsForward(int pwmVal) {
    Serial.println("Moving right motors forward");
    moveRightMotor1Forward(pwmVal);
    moveRightMotor2Forward(pwmVal);
}

void moveRightMotorsBackward(int pwmVal) {
    moveRightMotor1Backward(pwmVal);
    moveRightMotor2Backward(pwmVal);
}

// --- PID-Controlled Movement ---
void moveForward(int distance, int basePWM) {
    noInterrupts();
    left1Pos = left2Pos = right1Pos = right2Pos = 0;
    interrupts();
    
    long targetPulses = distance / distancePerPulse;
    unsigned long startTime = millis();

    leftSetpoint = rightSetpoint = targetPulses;
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    while (true) {
        noInterrupts();
        long leftPulses = (abs(left1Pos) + abs(left2Pos)) / 2;
        long rightPulses = (abs(right1Pos) + abs(right2Pos)) / 2;
        interrupts();

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

    noInterrupts();
    left1Pos = left2Pos = right1Pos = right2Pos = 0;
    interrupts();

    leftSetpoint = rightSetpoint = targetPulses;

    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    while (true) {
        noInterrupts();
        long leftPulses = (abs(left1Pos) + abs(left2Pos)) / 2;
        long rightPulses = (abs(right1Pos) + abs(right2Pos)) / 2;
        interrupts();

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

    noInterrupts();
    left1Pos = left2Pos = right1Pos = right2Pos = 0;
    interrupts();

    leftSetpoint = rightSetpoint = targetPulses;

    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    while (true) {
        noInterrupts();
        long leftPulses = (abs(left1Pos) + abs(left2Pos)) / 2;
        long rightPulses = (abs(right1Pos) + abs(right2Pos)) / 2;
        interrupts();

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

void moveForwardUntilObstacle() {
    if (!tof1.init()) {  // Ensure proper initialization of tof1
        Serial.println("ToF sensor initialization failed!");
        return;
    }

    // Reset encoders to track distance traveled
    noInterrupts();
    left1Pos = right1Pos = 0;
    interrupts();

    // Start moving forward
    digitalWrite(EnablePin1, HIGH);
    digitalWrite(EnablePin2, HIGH);  // Enable motors
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
    noInterrupts();
    long leftPulses = abs(left1Pos);
    long rightPulses = abs(right1Pos);
    interrupts();
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
    
    noInterrupts();
    long initialLeftPos = left1Pos;
    long initialRightPos = right1Pos;
    interrupts();

    moveBackward(150);

    while (true) {
        noInterrupts();
        long leftPulses = abs(left1Pos - initialLeftPos);
        long rightPulses = abs(right1Pos - initialRightPos);
        interrupts();
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
    digitalWrite(leftMotor1DirPin1, LOW);
    digitalWrite(leftMotor1DirPin2, LOW);
    digitalWrite(leftMotor2DirPin1, LOW);
    digitalWrite(leftMotor2DirPin2, LOW);
    digitalWrite(rightMotor1DirPin1, LOW);
    digitalWrite(rightMotor1DirPin2, LOW);
    digitalWrite(rightMotor2DirPin1, LOW);
    digitalWrite(rightMotor2DirPin2, LOW);
    analogWrite(leftMotor1PWMPin, 0);
    analogWrite(leftMotor2PWMPin, 0);
    analogWrite(rightMotor1PWMPin, 0);
    analogWrite(rightMotor2PWMPin, 0);
}
 */