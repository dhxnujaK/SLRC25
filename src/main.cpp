#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>

#define XSHUT_PIN 41

// Servo configuration
#define SERVO_FREQ 50  // Frequency for analog servos (Hz)
#define SERVOMIN  150  // Minimum pulse length count (out of 4096)
#define SERVOMAX  600  // Maximum pulse length count (out of 4096)

// Servo channels for robot arm joints
#define BASE_SERVO      0
#define ARM_SERVO       1
#define WRIST_SERVO     2
#define GRIPPER_SERVO   3

#define DEFAULT_SPEED 20  // Speed range: 1 (slow) to 100 (fast)

// --- MPU6050 Setup ---
MPU6050 mpu(Wire);

// --- Encoders (All 4) ---
Encoder encoderLeft1(34, 35);
Encoder encoderLeft2(19, 18); // PID-controlled
Encoder encoderRight1(26, 27);
Encoder encoderRight2(2, 3);  // PID-controlled

// --- Motor Pins ---
const int leftMotor1PWMPin = 10, leftMotor1DirPin1 = 31, leftMotor1DirPin2 = 30;
const int leftMotor2PWMPin = 9, leftMotor2DirPin1 = 32, leftMotor2DirPin2 = 33;
const int rightMotor1PWMPin = 12, rightMotor1DirPin1 = 38, rightMotor1DirPin2 = 39;
const int rightMotor2PWMPin = 11, rightMotor2DirPin1 = 22, rightMotor2DirPin2 = 23;

// --- Enable Pins ---
const int EnablePin1 = 25, EnablePin2 = 24;

// --- Robot Parameters ---
const float WHEEL_DIAMETER_CM = 5.4;
const int COUNTS_PER_REV = 100;
const float GEAR_RATIO = 15;
const float DISTANCE_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / (COUNTS_PER_REV * GEAR_RATIO);

int int_angles[4] = {90,90,90,90};
int drop_angles[4] = {90,110,0,98};
int current_angles[4] = {90,90,90,90};

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// Initialize the PWM driver at the default I2C address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Function to Set Motor Speeds ---
void moveForward(int speed, float distance);
void moveServoSmoothly(uint8_t servoNum, int target_angle, int speed = DEFAULT_SPEED);

void setMotorSpeed(int left1, int left2, int right1, int right2) {
  analogWrite(leftMotor1PWMPin, abs(left1*0.92));
  digitalWrite(leftMotor1DirPin1, left1 > 0);
  digitalWrite(leftMotor1DirPin2, left1 < 0);

  analogWrite(leftMotor2PWMPin, abs(left2));
  digitalWrite(leftMotor2DirPin1, left2 > 0);
  digitalWrite(leftMotor2DirPin2, left2 < 0);

  analogWrite(rightMotor1PWMPin, abs(right1));
  digitalWrite(rightMotor1DirPin1, right1 > 0);
  digitalWrite(rightMotor1DirPin2, right1 < 0);

  analogWrite(rightMotor2PWMPin, abs(right2));
  digitalWrite(rightMotor2DirPin1, right2 > 0);
  digitalWrite(rightMotor2DirPin2, right2 < 0);
}

void stopAllMotors() {
  setMotorSpeed(0, 0, 0, 0);
  Serial.println("Motors Stopped");
}

void moveBackward(int speed, float distance) {
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);
  setMotorSpeed(-speed, -speed, -speed, -speed);
  Serial.println("Moving Forward");
  
  while (true) {
    long right2Count = abs(encoderRight2.read());
    long left2Count = abs(encoderLeft2.read());
    float right2Dist = right2Count * DISTANCE_PER_COUNT;
    float left2Dist = left2Count * DISTANCE_PER_COUNT;
    float avgDist = (right2Dist + left2Dist) / 2.0;

    if (avgDist >= distance) {
      moveForward(90, 1);
      stopAllMotors();
      break;
    }
    delay(10);
  }
}

void moveForward(int speed) {
  Serial.println("Moving Forward");
  
  // Reset encoders at the start
  encoderRight2.write(0);
  encoderLeft2.write(0);

  while (true) {
    // Update encoder distances
    long right2Count = abs(encoderRight2.read());
    long left2Count = abs(encoderLeft2.read());
    float right2Dist = right2Count * DISTANCE_PER_COUNT;
    float left2Dist = left2Count * DISTANCE_PER_COUNT;
    float avgDist = (right2Dist + left2Dist) / 2.0;

    // Check for obstacles
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) { // Valid reading
      Serial.print("Distance (mm): ");
      Serial.println(measure.RangeMilliMeter);

      if (measure.RangeMilliMeter < 100) { // Obstacle detected
        stopAllMotors();
        Serial.println("Obstacle detected! Reversing...");
        moveServoSmoothly(BASE_SERVO, 60);
        delay(500);
        moveServoSmoothly(BASE_SERVO, 90);
        
        // Reverse the robot by the distance it traveled (avgDist)
        moveBackward(90, avgDist);
        break; // Exit the loop
      }
    } else {
      Serial.println("Sensor error or out of range");
    }
    setMotorSpeed(speed, speed, speed, speed);

    delay(10); // Small delay to avoid flooding I²C bus
  }
}

void moveForward(int speed, float distance) {
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);
  setMotorSpeed(speed, speed, speed, speed);
  Serial.println("Moving Forward");
  
  while (true) {
    long right2Count = abs(encoderRight2.read());
    long left2Count = abs(encoderLeft2.read());
    float right2Dist = right2Count * DISTANCE_PER_COUNT;
    float left2Dist = left2Count * DISTANCE_PER_COUNT;
    float avgDist = (right2Dist + left2Dist) / 2.0;

    if (avgDist >= distance) {
      stopAllMotors();
      break;
    }
    delay(10);
  }
}

void moveBackward(int speed) {
  setMotorSpeed(-speed, -speed, -speed, -speed);
  Serial.println("Moving Backward");
}


// --- TURN BASED ON GYROSCOPE ---
void turnLeft(int speed, float targetAngle) {
  targetAngle *= 2;
  mpu.update();
  float startYaw = mpu.getAngleZ();
  float targetYaw = startYaw + targetAngle;
  Serial.print("Turning Left: Target Yaw = ");
  Serial.println(targetAngle);

  setMotorSpeed(-speed, -speed, speed, speed);
  
  while (true) {
    mpu.update();
    float currentYaw = mpu.getAngleZ();
    Serial.print("Turning Left: Current Yaw = ");
    Serial.println(currentYaw);
    if (currentYaw >= targetYaw) {
      stopAllMotors();
      break;
    }
    delay(5);
  }
}

void turnRight(int speed, float targetAngle) {
  targetAngle *= 2;
  mpu.update();
  float startYaw = mpu.getAngleZ();
  Serial.print("Turning Right: Target Yaw = ");
  Serial.println(targetAngle);

  setMotorSpeed(speed, speed, -speed, -speed);
  
  while (true) {
    mpu.update();
    float currentYaw = mpu.getAngleZ();
    if (currentYaw <= startYaw - targetAngle) {
      stopAllMotors();
      break;
    }
    delay(5);
  }
}

// Function to move a servo to a position (0-180 degrees mapped to pulse length)
void moveServo(uint8_t servoNum, uint8_t angle) {
  // Map angle (0-180) to pulse length (SERVOMIN-SERVOMAX)
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulse);
}

void moveServoSmoothly(uint8_t servoNum, int target_angle, int speed = DEFAULT_SPEED) {
  target_angle = constrain(target_angle, 0, 180);  // Keep within servo range

  int start_angle = current_angles[servoNum]; // Get current position
  int angle_diff = abs(target_angle - start_angle);
  
  if (angle_diff == 0) return;  // No movement needed

  int step_delay = map(speed, 1, 100, 50, 5); // Map speed (1 slow, 100 fast)

  // Gradual movement loop
  for (int i = 1; i <= angle_diff; i++) {
    int intermediate_angle = start_angle + (target_angle > start_angle ? i : -i);
    uint16_t pulse = map(intermediate_angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoNum, 0, pulse);
    delay(step_delay);
  }

  // Update current angle
  current_angles[servoNum] = target_angle;
}

void moveServosToAngles(int servos[], int angles[], int num_servos, int speed = DEFAULT_SPEED) {
  for (int i = 0; i < num_servos; i++) {
    moveServoSmoothly(servos[i], angles[i], speed);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Reset the sensor via XSHUT pin
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW); // Hold in reset
  delay(10);
  digitalWrite(XSHUT_PIN, HIGH); // Release reset
  delay(10);
  Serial.println("Initializing VL53L0X...");
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X!");
    while (1);
  }
  Serial.println("VL53L0X ready!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize the MPU6050
  Serial.println("Initializing MPU6050...");
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 failed!");
    while (1);
  }
  Serial.println("Calibrating MPU6050...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Calibration Done!");

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
  pinMode(EnablePin1, OUTPUT);
  pinMode(EnablePin2, OUTPUT);
  
  digitalWrite(EnablePin1, HIGH);
  digitalWrite(EnablePin2, HIGH);

  moveServoSmoothly(BASE_SERVO, 90);
}

// --- MAIN LOOP ---
void loop() {
  /* turnLeft(120, 90);      // Turn left by 90 degrees
  delay(1000);
  moveForward(90, 30);   // Move forward 30 cm
  delay(500);
  turnRight(120, 90);      // Turn left by 90 degrees
  delay(500);
  moveForward(90, 30);   // Move forward 30 cm
  delay(500);
  moveBackward(90, 30);   // Move backward 30 cm
  delay(500);
  turnLeft(120, 90);      // Turn left by 90 degrees
  delay(500);
  moveForward(90, 30);   // Move forward 30 cm
  delay(500);
  turnRight(120, 90);      // Turn left by 90 degrees
  delay(500);
  moveForward(90, 60);   // Move forward 30 cm
  delay(500);
  moveBackward(90, 60);   // Move backward 30 cm
  delay(1000); */

  //moveForward(90);

  turnLeft(120,90);
  delay(500);
  moveForward(90, 30);   // Move forward 30 cm
  delay(500);
  turnRight(120, 90);      // Turn left by 90 degrees
  delay(500);
  moveForward(90);

  delay(1000);
}