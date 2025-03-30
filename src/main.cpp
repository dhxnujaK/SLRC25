#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>

// --- MPU6050 Gyroscope ---
MPU6050 mpu(Wire);
float yaw = 0;

// --- Encoders (All 4) ---
Encoder encoderLeft1(34, 35);
Encoder encoderLeft2(19, 18);
Encoder encoderRight1(26, 27);
Encoder encoderRight2(2, 3);

// --- Motor Pins ---
const int leftMotor1PWMPin = 10, leftMotor1DirPin1 = 31, leftMotor1DirPin2 = 30;
const int leftMotor2PWMPin = 9, leftMotor2DirPin1 = 32, leftMotor2DirPin2 = 33;
const int rightMotor1PWMPin = 12, rightMotor1DirPin1 = 38, rightMotor1DirPin2 = 39;
const int rightMotor2PWMPin = 11, rightMotor2DirPin1 = 22, rightMotor2DirPin2 = 23;

// --- Enable Pins ---
const int EnablePin1 = 25;
const int EnablePin2 = 24;

// --- Robot Parameters ---
const float WHEEL_DIAMETER_CM = 5.4;
const int COUNTS_PER_REVOLUTION = 100;
const float GEAR_RATIO = 15;
const float DISTANCE_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / (COUNTS_PER_REVOLUTION * GEAR_RATIO);

void setMotorSpeed(int pwmPin, int dirPin1, int dirPin2, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(dirPin1, speed > 0 ? HIGH : LOW);
  digitalWrite(dirPin2, speed < 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

void stopAllMotors() {
  setMotorSpeed(leftMotor1PWMPin, leftMotor1DirPin1, leftMotor1DirPin2, 0);
  setMotorSpeed(leftMotor2PWMPin, leftMotor2DirPin1, leftMotor2DirPin2, 0);
  setMotorSpeed(rightMotor1PWMPin, rightMotor1DirPin1, rightMotor1DirPin2, 0);
  setMotorSpeed(rightMotor2PWMPin, rightMotor2DirPin1, rightMotor2DirPin2, 0);
  Serial.println("Motors Stopped");
}

void moveForward(int speed, float distance) {
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);
  
  setMotorSpeed(leftMotor1PWMPin, leftMotor1DirPin1, leftMotor1DirPin2, speed);
  setMotorSpeed(leftMotor2PWMPin, leftMotor2DirPin1, leftMotor2DirPin2, speed);
  setMotorSpeed(rightMotor1PWMPin, rightMotor1DirPin1, rightMotor1DirPin2, speed);
  setMotorSpeed(rightMotor2PWMPin, rightMotor2DirPin1, rightMotor2DirPin2, speed);
  
  Serial.println("Moving Forward");

  while (true) {
    long right2Count = abs(encoderRight2.read());
    long left2Count = abs(encoderLeft2.read());

    float right2Dist = right2Count * DISTANCE_PER_COUNT;
    float left2Dist = left2Count * DISTANCE_PER_COUNT;

    float Dist = (right2Dist + left2Dist) / 2.0;
    if (Dist >= distance) {
      stopAllMotors();
      break;
    }
    delay(10);
  }
}

// --- Gyroscope-Based Turn ---
void resetYaw() {
  Serial.println("Resetting Yaw...");
  mpu.update();
  yaw = 0;
}

void turnLeft(int speed, float angle) {
  resetYaw();

  setMotorSpeed(leftMotor1PWMPin, leftMotor1DirPin1, leftMotor1DirPin2, -speed);
  setMotorSpeed(leftMotor2PWMPin, leftMotor2DirPin1, leftMotor2DirPin2, -speed);
  setMotorSpeed(rightMotor1PWMPin, rightMotor1DirPin1, rightMotor1DirPin2, speed);
  setMotorSpeed(rightMotor2PWMPin, rightMotor2DirPin1, rightMotor2DirPin2, speed);
  
  Serial.print("Turning Left: Target Angle = ");
  Serial.println(angle);

  while (yaw < angle) {
    mpu.update();
    yaw = mpu.getAngleZ();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    delay(10);
  }
  
  stopAllMotors();
  Serial.println("Left Turn Complete");
}

void turnRight(int speed, float angle) {
  resetYaw();

  setMotorSpeed(leftMotor1PWMPin, leftMotor1DirPin1, leftMotor1DirPin2, speed);
  setMotorSpeed(leftMotor2PWMPin, leftMotor2DirPin1, leftMotor2DirPin2, speed);
  setMotorSpeed(rightMotor1PWMPin, rightMotor1DirPin1, rightMotor1DirPin2, -speed);
  setMotorSpeed(rightMotor2PWMPin, rightMotor2DirPin1, rightMotor2DirPin2, -speed);
  
  Serial.print("Turning Right: Target Angle = ");
  Serial.println(angle);

  while (yaw > -angle) {
    mpu.update();
    yaw = mpu.getAngleZ();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    delay(10);
  }
  
  stopAllMotors();
  Serial.println("Right Turn Complete");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Robot...");

  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 Initialization Failed! Error Code: ");
    Serial.println(status);
    while (1);
  }
  mpu.calcOffsets();  // Auto-calibrate MPU6050
  Serial.println("MPU6050 Ready!");

  // Set motor pins as output
  int motorPins[] = {leftMotor1PWMPin, leftMotor1DirPin1, leftMotor1DirPin2, leftMotor2PWMPin, leftMotor2DirPin1, leftMotor2DirPin2,
                     rightMotor1PWMPin, rightMotor1DirPin1, rightMotor1DirPin2, rightMotor2PWMPin, rightMotor2DirPin1, rightMotor2DirPin2};

  for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

  // Enable motor drivers
  pinMode(EnablePin1, OUTPUT);
  pinMode(EnablePin2, OUTPUT);
  digitalWrite(EnablePin1, HIGH);
  digitalWrite(EnablePin2, HIGH);

  Serial.println("Robot Ready!");
}

void loop() {
  moveForward(100, 30);  // Move 30 cm forward
  delay(1000);

  turnLeft(120, 90);  // Turn left by 90 degrees
  delay(1000);

  moveForward(100, 30);  // Move 30 cm forward
  delay(1000);

  turnRight(120, 90);  // Turn right by 90 degrees
  delay(1000);

  stopAllMotors();

}
