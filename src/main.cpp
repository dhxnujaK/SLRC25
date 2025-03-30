#include <Encoder.h>

// Initialize Encoders with specified pins
Encoder encoderLeft1(34, 35);
Encoder encoderLeft2(37, 36);
Encoder encoderRight1(26, 27);
Encoder encoderRight2(2, 3);

// Motor Control Pins
// Left Motor 1
const int leftMotor1PWMPin = 10;
const int leftMotor1DirPin1 = 31;
const int leftMotor1DirPin2 = 30;

// Left Motor 2
const int leftMotor2PWMPin = 9;
const int leftMotor2DirPin1 = 32;
const int leftMotor2DirPin2 = 33;

// Right Motor 1
const int rightMotor1PWMPin = 12;
const int rightMotor1DirPin1 = 38;
const int rightMotor1DirPin2 = 39;

// Right Motor 2
const int rightMotor2PWMPin = 11;
const int rightMotor2DirPin1 = 22;
const int rightMotor2DirPin2 = 23;

// Enable Pins
const int EnablePin1 = 25;
const int EnablePin2 = 24;

// Robot parameters (ADJUST THESE VALUES)
const float WHEEL_DIAMETER_CM = 6.5;      // Measure your wheel diameter
const int COUNTS_PER_REVOLUTION = 330;    // From encoder/motor specs
const float GEAR_RATIO = 1.0;             // If using gear reduction

// Calculated constants
const float DISTANCE_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / (COUNTS_PER_REVOLUTION * GEAR_RATIO);

// Encoder positions
volatile long left1Pos = 0;
volatile long left2Pos = 0;
volatile long right1Pos = 0;
volatile long right2Pos = 0;

// Motor control functions
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
  analogWrite(leftMotor2PWMPin, abs(speed));
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

void setup() {
  // Initialize motor control pins
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

  // Reset encoders to zero position
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);

  Serial.begin(115200);
}

void loop() {
  // Update encoder readings
  noInterrupts();  // Disable interrupts to read encoder values safely
  long newLeft1 = encoderLeft1.read();
  long newLeft2 = encoderLeft2.read();
  long newRight1 = encoderRight1.read();
  long newRight2 = encoderRight2.read();
  interrupts();  // Re-enable interrupts

  // Calculate distances
  float left1Dist = newLeft1 * DISTANCE_PER_COUNT;
  float left2Dist = newLeft2 * DISTANCE_PER_COUNT;
  float right1Dist = newRight1 * DISTANCE_PER_COUNT;
  float right2Dist = newRight2 * DISTANCE_PER_COUNT;

  // Check if positions have changed
  if (newLeft1 != left1Pos || newLeft2 != left2Pos || 
      newRight1 != right1Pos || newRight2 != right2Pos) {
    left1Pos = newLeft1;
    left2Pos = newLeft2;
    right1Pos = newRight1;
    right2Pos = newRight2;

    // Print distances in centimeters
    Serial.print("Distances (cm) | L1:");
    Serial.print(left1Dist, 2);  // 2 decimal places
    Serial.print(" L2:");
    Serial.print(left2Dist, 2);
    Serial.print(" R1:");
    Serial.print(right1Dist, 2);
    Serial.print(" R2:");
    Serial.println(right2Dist, 2);
  }

  setLeftMotor1Speed(50);  // Example speed for left motor 1
  setLeftMotor2Speed(50);  // Example speed for left motor 2
  setRightMotor1Speed(50); // Example speed for right motor 1
  setRightMotor2Speed(50); // Example speed for right motor 2

  delay(5000);
  setRightMotor1Speed(-50); // Example speed for right motor 1
  setRightMotor2Speed(-50);
  setLeftMotor1Speed(-50); // Example speed for right motor 1
  setLeftMotor2Speed(-50);  // Example speed for left motor 2

  // Add small delay to prevent serial overflow
  delay(5000);
}