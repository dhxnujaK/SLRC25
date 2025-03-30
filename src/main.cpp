#include <Encoder.h>
#include <Adafruit_MCP23X17.h>

// Initialize Encoders with specified pins
Encoder encoderLeft1(34, 35);
Encoder encoderLeft2(19, 18);
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
const float WHEEL_DIAMETER_CM = 4.5;      // Measure your wheel diameter
const int COUNTS_PER_REVOLUTION = 100;    // From encoder/motor specs
const float GEAR_RATIO = 15;             // If using gear reduction

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

void stopMotors() {
  setLeftMotor1Speed(0);
  setLeftMotor2Speed(0);
  setRightMotor1Speed(0);
  setRightMotor2Speed(0);
}

void moveForward(float distance_cm, int speed = 50) {
  // Reset encoders
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);
  
  // Calculate target counts
  long target_counts = distance_cm / DISTANCE_PER_COUNT;
  
  // Start moving forward
  setLeftMotor1Speed(speed);
  setLeftMotor2Speed(speed);
  setRightMotor1Speed(speed);
  setRightMotor2Speed(speed);
  
  // Wait until target distance is reached
  while (true) {
    noInterrupts();
    long current_left1 = encoderLeft1.read();
    long current_left2 = encoderLeft2.read();
    long current_right1 = encoderRight1.read();
    long current_right2 = encoderRight2.read();
    interrupts();
    
    // Check if any motor has reached the target
    if (abs(current_left1) >= target_counts || 
        abs(current_left2) >= target_counts || 
        abs(current_right1) >= target_counts || 
        abs(current_right2) >= target_counts) {
      break;
    }
    delay(10); // Small delay to prevent CPU overload
  }
  
  // Stop motors
  stopMotors();
}

void moveBackward(float distance_cm, int speed = 50) {
  // Reset encoders
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);
  
  // Calculate target counts
  long target_counts = distance_cm / DISTANCE_PER_COUNT;
  
  // Start moving backward
  setLeftMotor1Speed(-speed);
  setLeftMotor2Speed(-speed);
  setRightMotor1Speed(-speed);
  setRightMotor2Speed(-speed);
  
  // Wait until target distance is reached
  while (true) {
    noInterrupts();
    long current_left1 = encoderLeft1.read();
    long current_left2 = encoderLeft2.read();
    long current_right1 = encoderRight1.read();
    long current_right2 = encoderRight2.read();
    interrupts();
    
    // Check if any motor has reached the target
    if (abs(current_left1) >= target_counts || 
        abs(current_left2) >= target_counts || 
        abs(current_right1) >= target_counts || 
        abs(current_right2) >= target_counts) {
      break;
    }
    delay(10); // Small delay to prevent CPU overload
  }
  
  // Stop motors
  stopMotors();
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

  // Example usage in loop (you can remove or modify this)
  /* static bool hasMoved = false;
  if (!hasMoved) {
    moveForward(20.0);  // Move forward 20 cm
    delay(1000);
    moveBackward(20.0); // Move backward 20 cm
    hasMoved = true;
  } */

  moveForward(20.0);  // Move forward 20 cm
    delay(1000);
    moveBackward(20.0);

  // Add small delay to prevent serial overflow
  delay(1000);
}