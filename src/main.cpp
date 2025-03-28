#include <Arduino.h>
#include <Encoder.h>
#include "config.h"
#include "motor_control.h"

void setup() {
  Serial.begin(9600);

  // --- Set up motor driver pins ---
  // Left motors:
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);

  // Right motors:
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);

  pinMode(EnablePin, OUTPUT);

  // No additional encoder setup is needed when using the Encoder library.
}

void loop() {
  // For demonstration, rotate the robot by 90° once.
  digitalWrite(EnablePin, HIGH);
  /* static bool rotated = false;
  if (!rotated) {
    rotateRobot(90.0); // Rotate 90 degrees (positive for right turn, negative for left)
    rotated = true;
  } */

  // Retrieve current encoder positions
  long leftCount1  = encoderLeft1.read();
  long leftCount2  = encoderLeft2.read();
  long rightCount1 = encoderRight1.read();
  long rightCount2 = encoderRight2.read();

  // Average counts per side:
  long leftCount  = (leftCount1 + leftCount2) / 2;
  long rightCount = (rightCount1 + rightCount2) / 2;

  // Convert counts to distance traveled (meters)
  float leftDistance  = leftCount * distancePerPulse;
  float rightDistance = rightCount * distancePerPulse;

/*   Serial.print("Left Distance: ");
  Serial.print(leftDistance, 4);
  Serial.print(" m, Right Distance: ");
  Serial.print(rightDistance, 4);
  Serial.println(" m"); */
  Serial.print("Left Count: ");
  Serial.print(leftCount,4);
  Serial.println();

  delay(100);
}


