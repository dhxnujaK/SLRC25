#include <Encoder.h>

// --- Encoder Setup ---
// Left motors
extern Encoder encoderLeft1;    // Left Motor 1 encoder
extern Encoder encoderLeft2;    // Left Motor 2 encoder

// Right motors
extern Encoder encoderRight1;   // Right Motor 1 encoder
extern Encoder encoderRight2;   // Right Motor 2 encoder

// --- Motor Driver Pins (TB6612FNG) ---
// Left Motor 1:
const int leftMotor1PWMPin   = 10;
const int leftMotor1DirPin1  = 30;
const int leftMotor1DirPin2  = 31;

// Left Motor 2:
const int leftMotor2PWMPin   = 9;
const int leftMotor2DirPin1  = 32;
const int leftMotor2DirPin2  = 33;

// Right Motor 1:
const int rightMotor1PWMPin  = 12;
const int rightMotor1DirPin1 = 38;
const int rightMotor1DirPin2 = 39;

// Right Motor 2:
const int rightMotor2PWMPin  = 11;
const int rightMotor2DirPin1 = 23;
const int rightMotor2DirPin2 = 22;

// --- Driver Enable Pins ---
const int EnablePin1 = 25;
const int EnablePin2 = 24;

// --- Motor Control Function ---
void setMotor(int pwmPin, int dirPin1, int dirPin2, int speed) {
  speed = constrain(speed, -255, 255); // Constrain speed to PWM range
  
  if (speed > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  } else if (speed < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}

// --- Setup Function ---
void setup() {
  Serial.begin(9600);
  
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
}

// --- Helper Functions for Movement ---
void setLeftMotors(int speed) {
  setMotor(leftMotor1PWMPin, leftMotor1DirPin1, leftMotor1DirPin2, speed);
  setMotor(leftMotor2PWMPin, leftMotor2DirPin1, leftMotor2DirPin2, speed);
}

void setRightMotors(int speed) {
  setMotor(rightMotor1PWMPin, rightMotor1DirPin1, rightMotor1DirPin2, speed);
  setMotor(rightMotor2PWMPin, rightMotor2DirPin1, rightMotor2DirPin2, speed);
}

void stopAllMotors() {
  setLeftMotors(0);
  setRightMotors(0);
}

// --- Main Loop ---
void loop() {
  // Read encoder values
  long encL1 = encoderLeft1.read();
  long encL2 = encoderLeft2.read();
  long encR1 = encoderRight1.read();
  long encR2 = encoderRight2.read();

  // Display encoder values
  Serial.print("Encoders - L1:");
  Serial.print(encL1);
  Serial.print(" L2:");
  Serial.print(encL2);
  Serial.print(" R1:");
  Serial.print(encR1);
  Serial.print(" R2:");
  Serial.println(encR2);

  // Example movement pattern
  setLeftMotors(200);   // Forward left
  setRightMotors(200);  // Forward right
  delay(2000);
  
  stopAllMotors();
  delay(1000);
  
  setLeftMotors(-150);  // Reverse left
  setRightMotors(-150); // Reverse right
  delay(2000);
  
  stopAllMotors();
  delay(1000);
  
  // Differential steering
  setLeftMotors(-150);  // Turn right
  setRightMotors(150);
  delay(1000);
  
  stopAllMotors();
  delay(1000);
}