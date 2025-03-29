#include <Encoder.h>

// Initialize Encoders with specified pins
Encoder encoderLeft1(34, 35);
Encoder encoderLeft2(37, 36);
Encoder encoderRight1(27, 26);
Encoder encoderRight2(29, 28);

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

// Encoder positions
volatile long left1Pos = 0;
volatile long left2Pos = 0;
volatile long right1Pos = 0;
volatile long right2Pos = 0;

// Motor control functions
void setLeftMotor1Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(leftMotor1DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(leftMotor1DirPin2, speed > 0 ? LOW : HIGH);
  analogWrite(leftMotor1PWMPin, abs(speed));
}

void setLeftMotor2Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(leftMotor2DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(leftMotor2DirPin2, speed > 0 ? LOW : HIGH);
  analogWrite(leftMotor2PWMPin, abs(speed));
}

void setRightMotor1Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(rightMotor1DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(rightMotor1DirPin2, speed > 0 ? LOW : HIGH);
  analogWrite(rightMotor1PWMPin, abs(speed));
}

void setRightMotor2Speed(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(rightMotor2DirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(rightMotor2DirPin2, speed > 0 ? LOW : HIGH);
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

  Serial.begin(115200);
}

void loop() {
  // Update encoder readings
  long newLeft1 = encoderLeft1.read();
  long newLeft2 = encoderLeft2.read();
  long newRight1 = encoderRight1.read();
  long newRight2 = encoderRight2.read();

  // Check if positions have changed
  if (newLeft1 != left1Pos || newLeft2 != left2Pos || 
      newRight1 != right1Pos || newRight2 != right2Pos) {
    left1Pos = newLeft1;
    left2Pos = newLeft2;
    right1Pos = newRight1;
    right2Pos = newRight2;

    // Print all encoder values
    Serial.print("Encoders - L1:");
    Serial.print(left1Pos);
    Serial.print(" L2:");
    Serial.print(left2Pos);
    Serial.print(" R1:");
    Serial.print(right1Pos);
    Serial.print(" R2:");
    Serial.println(right2Pos);
  }
  // Test code - move all motors forward at 50% speed
/* setLeftMotor1Speed(20);
setLeftMotor2Speed(20);
setRightMotor1Speed(20);
setRightMotor2Speed(20); */

  // Add small delay to prevent serial overflow
  delay(10);
}

