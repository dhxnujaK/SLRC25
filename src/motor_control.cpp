#include "motor_control.h"
#include "config.h"
#include <Arduino.h>

// --- Function to drive motors ---
// leftDir/rightDir: 1 for forward, -1 for reverse, 0 for stop
// pwmVal: speed value (0-255)
void driveMotors(int leftDir, int rightDir, int pwmVal) {
    // Left motors
    if (leftDir > 0) {
      digitalWrite(leftMotorDirPin1, HIGH);
      digitalWrite(leftMotorDirPin2, LOW);
    } else if (leftDir < 0) {
      digitalWrite(leftMotorDirPin1, LOW);
      digitalWrite(leftMotorDirPin2, HIGH);
    } else {
      digitalWrite(leftMotorDirPin1, LOW);
      digitalWrite(leftMotorDirPin2, LOW);
    }
    analogWrite(leftMotorPWMPin, pwmVal);
  
    // Right motors
    if (rightDir > 0) {
      digitalWrite(rightMotorDirPin1, HIGH);
      digitalWrite(rightMotorDirPin2, LOW);
    } else if (rightDir < 0) {
      digitalWrite(rightMotorDirPin1, LOW);
      digitalWrite(rightMotorDirPin2, HIGH);
    } else {
      digitalWrite(rightMotorDirPin1, LOW);
      digitalWrite(rightMotorDirPin2, LOW);
    }
    analogWrite(rightMotorPWMPin, pwmVal);
  }

  // --- Function to rotate the robot by a given angle (degrees) ---
// Positive angle: right turn (left motors forward, right motors reverse)
// Negative angle: left turn (left motors reverse, right motors forward)
void rotateRobot(float angleDegrees) {
    // Reset all encoder counts
    encoderLeft1.write(0);
    encoderLeft2.write(0);
    encoderRight1.write(0);
    encoderRight2.write(0);
  
    // Calculate the distance each wheel must travel for an in-place rotation.
    // For in-place rotation: distance = PI * axleLength * (|angle| / 360)
    float distancePerWheel = 3.1416 * axleLength * fabs(angleDegrees) / 360.0;
    long targetPulses = distancePerWheel / distancePerPulse;
  
    Serial.print("Rotating ");
    Serial.print(angleDegrees);
    Serial.print("° (Target pulses per wheel: ");
    Serial.print(targetPulses);
    Serial.println(")");
  
    // Determine motor directions for an in-place turn:
    int leftMotorDirection  = (angleDegrees > 0) ? 1 : -1;
    int rightMotorDirection = (angleDegrees > 0) ? -1 : 1;
  
    // Start rotation at a moderate speed (adjust PWM as needed)
    driveMotors(leftMotorDirection, rightMotorDirection, 150);
  
    // Continue rotating until the average encoder count meets or exceeds the target.
    while (true) {
      long lCount1  = abs(encoderLeft1.read());
      long lCount2  = abs(encoderLeft2.read());
      long rCount1  = abs(encoderRight1.read());
      long rCount2  = abs(encoderRight2.read());
  
      long leftAvg  = (lCount1 + lCount2) / 2;
      long rightAvg = (rCount1 + rCount2) / 2;
      long avgCount = (leftAvg + rightAvg) / 2;
  
      if (avgCount >= targetPulses) {
        break;
      }
      delay(10);  // small delay to prevent locking up the CPU completely
    }
  
    // Stop the motors
    driveMotors(0, 0, 0);
    Serial.println("Rotation complete.");
  }