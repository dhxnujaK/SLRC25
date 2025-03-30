#include <Encoder.h>

// --- Encoders (All 4) ---
Encoder encoderLeft1(34, 35);   // Left1
Encoder encoderLeft2(19, 18);   // Left2 (PID-controlled)
Encoder encoderRight1(26, 27);  // Right1
Encoder encoderRight2(2, 3);    // Right2 (PID-controlled)

// --- Motor Pins (All 4) ---
// Left Motor 1
const int leftMotor1PWMPin = 10;
const int leftMotor1DirPin1 = 31;
const int leftMotor1DirPin2 = 30;

// Left Motor 2 (PID-controlled)
const int leftMotor2PWMPin = 9;
const int leftMotor2DirPin1 = 32;
const int leftMotor2DirPin2 = 33;

// Right Motor 1
const int rightMotor1PWMPin = 12;
const int rightMotor1DirPin1 = 38;
const int rightMotor1DirPin2 = 39;

// Right Motor 2 (PID-controlled)
const int rightMotor2PWMPin = 11;
const int rightMotor2DirPin1 = 22;
const int rightMotor2DirPin2 = 23;

// --- Enable Pins ---
const int EnablePin1 = 25;  // Left motors
const int EnablePin2 = 24;  // Right motors

// --- Robot Parameters ---
const float WHEEL_DIAMETER_CM = 4.5;      // Measure your wheel
const int COUNTS_PER_REVOLUTION = 100;    // From encoder specs
const float GEAR_RATIO = 15;              // Gear reduction ratio
const float DISTANCE_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / (COUNTS_PER_REVOLUTION * GEAR_RATIO);

void setLeftMotor1Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(leftMotor1DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(leftMotor1DirPin2, speed < 0 ? HIGH : LOW);
  analogWrite(leftMotor1PWMPin, abs(speed));
}

void setLeftMotor2Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(leftMotor2DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(leftMotor2DirPin2, speed < 0 ? HIGH : LOW);
  analogWrite(leftMotor2PWMPin, abs(speed*1.0923));
}

void setRightMotor1Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(rightMotor1DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(rightMotor1DirPin2, speed < 0 ? HIGH : LOW);
  analogWrite(rightMotor1PWMPin, abs(speed));
}

void setRightMotor2Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(rightMotor2DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(rightMotor2DirPin2, speed < 0 ? HIGH : LOW);
  analogWrite(rightMotor2PWMPin, abs(speed));
}

void stopAllMotors() {
  setLeftMotor1Speed(0);
  setLeftMotor2Speed(0);
  setRightMotor1Speed(0);
  setRightMotor2Speed(0);
  Serial.println("Motors Stopped");
}

void moveForward(int speed) {
  setLeftMotor1Speed(speed);
  setLeftMotor2Speed(speed);
  setRightMotor1Speed(speed);
  setRightMotor2Speed(speed);
  Serial.println("Moving Forward");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Encoder Reading Started");
  pinMode(leftMotor1PWMPin, OUTPUT);
  pinMode(leftMotor1DirPin1, OUTPUT);
  pinMode(leftMotor1DirPin2, OUTPUT);
  
  pinMode(leftMotor2PWMPin, OUTPUT);
  pinMode(leftMotor2DirPin1, OUTPUT);
  pinMode(leftMotor2DirPin2, OUTPUT);
  
  pinMode(rightMotor1PWMPin, OUTPUT);
  pinMode(rightMotor1DirPin1, OUTPUT);
  pinMode(rightMotor1DirPin2, OUTPUT);
  
  pinMode(rightMotor2PWMPin, OUTPUT);
  pinMode(rightMotor2DirPin1, OUTPUT);
  pinMode(rightMotor2DirPin2, OUTPUT);

  // Enable motor drivers
  pinMode(EnablePin1, OUTPUT);
  pinMode(EnablePin2, OUTPUT);
  digitalWrite(EnablePin1, HIGH);
  digitalWrite(EnablePin2, HIGH);
}

void loop() {
  // Read raw encoder counts
  long left1Count = encoderLeft1.read();
  long left2Count = encoderLeft2.read();
  long right1Count = encoderRight1.read();
  long right2Count = encoderRight2.read();

  // Calculate distances in centimeters
  float left1Dist = left1Count * DISTANCE_PER_COUNT;
  float left2Dist = left2Count * DISTANCE_PER_COUNT;
  float right1Dist = right1Count * DISTANCE_PER_COUNT;
  float right2Dist = right2Count * DISTANCE_PER_COUNT;

  // Print all encoder values to serial monitor
  Serial.print("Left1: ");
  Serial.print(left1Count);
  Serial.print(" (");
  Serial.print(left1Dist);
  Serial.print(" cm)\t");

  Serial.print("Left2: ");
  Serial.print(left2Count);
  Serial.print(" (");
  Serial.print(left2Dist);
  Serial.print(" cm)\t");

  Serial.print("Right1: ");
  Serial.print(right1Count);
  Serial.print(" (");
  Serial.print(right1Dist);
  Serial.print(" cm)\t");

  Serial.print("Right2: ");
  Serial.print(right2Count);
  Serial.print(" (");
  Serial.print(right2Dist);
  Serial.println(" cm)");

  // Short delay to prevent serial overflow
  moveForward(50);  // Move forward at speed 100
  delay(1000);      // Move for 1 second
  stopAllMotors();  // Stop motors
  delay(1000);
}