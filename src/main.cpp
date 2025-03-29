#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "config.h"
#include "motor_control.h"

VL53L0X tof1;

void setup() {
  Serial.begin(9600);

  pinMode(TOF1_XSHUT, OUTPUT);
  digitalWrite(TOF1_XSHUT, LOW);
  delay(10);
  digitalWrite(TOF1_XSHUT, HIGH);
  delay(10);
  tof1.setAddress(0x30);
  // --- Set up motor driver pins ---
  // Left Motor 1:
  pinMode(leftMotor1PWMPin, OUTPUT);
  pinMode(leftMotor1DirPin1, OUTPUT);
  pinMode(leftMotor1DirPin2, OUTPUT);

  // Left Motor 2:
  pinMode(leftMotor2PWMPin, OUTPUT);
  pinMode(leftMotor2DirPin1, OUTPUT);
  pinMode(leftMotor2DirPin2, OUTPUT);

  // Right Motor 1:
  pinMode(rightMotor1PWMPin, OUTPUT);
  pinMode(rightMotor1DirPin1, OUTPUT);
  pinMode(rightMotor1DirPin2, OUTPUT);

  // Right Motor 2:
  pinMode(rightMotor2PWMPin, OUTPUT);
  pinMode(rightMotor2DirPin1, OUTPUT);
  pinMode(rightMotor2DirPin2, OUTPUT);

  pinMode(EnablePin1, OUTPUT);
  pinMode(EnablePin2, OUTPUT);
  digitalWrite(EnablePin1, HIGH);
  digitalWrite(EnablePin2, HIGH);

  // --- TOF Sensor Setup ---
  Wire.begin();

  // Assign unique addresses to multiple sensors
  pinMode(TOF1_XSHUT, OUTPUT);

  digitalWrite(TOF1_XSHUT, LOW);
  delay(10);

  digitalWrite(TOF1_XSHUT, HIGH);
  delay(10);
  tof1.setAddress(0x30);  // Assign new address

  tof1.init();
  if (!tof1.init()) {
    Serial.println("ToF sensor initialization failed!");
  }else{
    Serial.println("ToF sensor initialized successfully!");
  }
}

void loop() {

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
  /* float leftDistance  = leftCount * distancePerPulse;
  float rightDistance = rightCount * distancePerPulse; */

/*   Serial.print("Left Distance: ");
  Serial.print(leftDistance, 4);
  Serial.print(" m, Right Distance: ");
  Serial.print(rightDistance, 4);
  Serial.println(" m"); */
  Serial.print("Left Count: ");
  Serial.println(leftCount,4);
  Serial.print("Right Count: ");
  Serial.println(rightCount,4); 

  /* rotateLeft(90,100);*/
  //moveForwardUntilObstacle();

  //moveForward(1, 150); // Move forward for 0.5 meters at base PWM of 100
  //stopMotors(); // Stop motors
  //delay(1000); // Wait for 1 second */
  //moveRightMotorsBackward(150);

  delay(1000);
}


