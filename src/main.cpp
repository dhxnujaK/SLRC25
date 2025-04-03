#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define XSHUT_PIN 41
#define S0 46
#define S1 47
#define S2 42
#define S3 44
#define OUT 43

#define S0_arm 28
#define S1_arm 26
#define S2_arm 35
#define S3_arm 34
#define OUT_arm 29

#define LED 13   // Pin for LED

// Set color scale (you can adjust these based on your requirements)
#define SCALE 1000  // Adjust this value as necessary

// EEPROM locations to store calibration values
#define EEPROM_WHITE_RED 0
#define EEPROM_WHITE_GREEN 1
#define EEPROM_WHITE_BLUE 2
#define EEPROM_ORANGE_RED 3
#define EEPROM_ORANGE_GREEN 4
#define EEPROM_ORANGE_BLUE 5

// Calibration time in milliseconds (2 minutes = 120000 ms)
#define CALIBRATION_TIME_LIMIT 10000

// Servo configuration
#define SERVO_FREQ 50  // Frequency for analog servos (Hz)
#define SERVOMIN  150  // Minimum pulse length count (out of 4096)
#define SERVOMAX  600  // Maximum pulse length count (out of 4096)

#define SERVOMIN_1  102  // Minimum pulse length count (out of 4096)
#define SERVOMAX_1  492  // Maximum pulse length count (out of 4096)

// Servo channels for robot arm joints
#define BASE_SERVO      0
#define ARM_SERVO       1
#define WRIST_SERVO     4
#define GRIPPER_SERVO   3

#define DEFAULT_SPEED 20  // Speed range: 1 (slow) to 100 (fast)

int int_angles[4] = {90,90,90,90};
int drop_angles[4] = {90,110,0,98};
int current_angles[4] = {90,90,90,90};

// --- MPU6050 Setup ---
MPU6050 mpu(Wire);
// Initialize the PWM driver at the default I2C address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

// Variables to store pulse width measurements
int redValue, greenValue, blueValue;

int redVal_arm, greenVal_arm, blueVal_arm;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

unsigned long calibrationStartTime;

// Function declarations
void startCalibration();
void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue, int samples = 10);
void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue);
void performTask();

// --- Function to Set Motor Speeds ---
void moveForward(int speed, float distance);
void readRGB();
void grab_ball();

void LEDblink(){
  digitalWrite(LED,HIGH);
}
void LEDnoblink(){
  digitalWrite(LED,LOW);
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

void moveServoSmoothlyUpper(uint8_t servoNum, int target_angle, int speed = DEFAULT_SPEED) {
  target_angle = constrain(target_angle, 0, 180);  

  int start_angle = current_angles[servoNum];
  int angle_diff = abs(target_angle - start_angle);

  if (angle_diff == 0) return;  // No need to move

  int step_delay = map(speed, 1, 100, 50, 5); // Speed mapping

  for (int i = 1; i <= angle_diff; i++) {
      int intermediate_angle = start_angle + (target_angle > start_angle ? i : -i);
      uint16_t pulse = map(intermediate_angle, 0, 180, SERVOMIN_1, SERVOMAX_1);
      pwm.setPWM(servoNum, 0, pulse);
      delay(step_delay);
  }

  current_angles[servoNum] = target_angle;
}


void moveServosToAngles(int servos[], int angles[], int num_servos, int speed = DEFAULT_SPEED) {
  for (int i = 0; i < num_servos; i++) {
    moveServoSmoothly(servos[i], angles[i], speed);
  }
}

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

void moveUntillGreen(int speed) {
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);
  Serial.println("Moving Forward");

  while (true) {
        // Update encoder distances
        long right2Count = abs(encoderRight2.read());
        long left2Count = abs(encoderLeft2.read());
        float right2Dist = right2Count * DISTANCE_PER_COUNT;
        float left2Dist = left2Count * DISTANCE_PER_COUNT;
        float avgDist = (right2Dist + left2Dist) / 2.0;
    readRGB();
    Serial.print("Red: ");
    Serial.print(redValue);
    Serial.print(" Green: ");
    Serial.print(greenValue);
    Serial.print(" Blue: ");
    Serial.println(blueValue);

    if ((greenValue < redValue && greenValue < blueValue) && (redValue+blueValue+greenValue)/3 > 100) {
      stopAllMotors();
      Serial.println("Green detect56ed! Stopping...");
      grab_ball();
      moveBackward(90,avgDist);
      break;
    }
    setMotorSpeed(speed, speed, speed, speed);
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
    delay(10);
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
    delay(10);
  }
}

// Function to read RGB values from TCS230
void readRGB() {
  // Measure Red component
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redValue = pulseIn(OUT, LOW); // Higher pulse width = lower intensity

  // Measure Green component
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenValue = pulseIn(OUT, LOW);

  // Measure Blue component
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueValue = pulseIn(OUT, LOW);
}

// servo function to move the gripper
void grab_ball() {
  moveServoSmoothly(GRIPPER_SERVO, 98);
  moveServoSmoothly(BASE_SERVO, 90); delay(500);
  moveServoSmoothly(ARM_SERVO,30); delay(500);
  moveServoSmoothlyUpper(WRIST_SERVO,30); delay(500);
  moveServoSmoothly(GRIPPER_SERVO,68); delay(500);

  moveServoSmoothly(ARM_SERVO, 116);
  moveServoSmoothlyUpper(WRIST_SERVO, 5);
  moveServoSmoothly(GRIPPER_SERVO, 98); delay(500);
  
}
bool isRun = false;
// --- SETUP ---
void setup() {

  if ((millis() - calibrationStartTime > CALIBRATION_TIME_LIMIT) || !isRun) {
    // After calibration time window, we can no longer calibrate, so we perform the task
    performTask();
    isRun = true;
  }

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

  // Initialize the PWM driver
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

  pinMode(LED, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  //for arm color sensor
  pinMode(S0_arm, OUTPUT);
  pinMode(S1_arm, OUTPUT);
  pinMode(S2_arm, OUTPUT);
  pinMode(S3_arm, OUTPUT);
  pinMode(OUT_arm, INPUT);

  // Set frequency scaling to 20% (recommended for better resolution)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  digitalWrite(EnablePin1, HIGH);
  digitalWrite(EnablePin2, HIGH);

  // Initialize all servos to a neutral position
  moveServoSmoothly(BASE_SERVO, 90); delay(1000);
  moveServoSmoothly(ARM_SERVO, 116); delay(1000);
  moveServoSmoothlyUpper(WRIST_SERVO, 5); delay(1000);
  moveServoSmoothly(GRIPPER_SERVO, 98); delay(1000);

  Serial.println("Base servo moved to 90 degrees.");

/*   // Configure frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH); */
  
  // Give some time for sensor initialization
  delay(500);
  
  // Start the calibration process
  startCalibration();

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

/*   turnLeft(120,90);
  delay(500);
  moveForward(90, 30);   // Move forward 30 cm
  delay(500);
  turnRight(120, 90);      // Turn left by 90 degrees
  delay(500);
  moveForward(90); */

  turnLeft(120,90);
  delay(500);
  moveForward(90,30);
  delay(500);
  turnRight(120,90);
  delay(500);
  moveUntillGreen(90); // Move until green is detected

  delay(1000);
}


void startCalibration() {
  // Start time measurement for calibration window
  calibrationStartTime = millis(); // Record the starting time

  // Calibration values for White and Orange
  int whiteRed, whiteGreen, whiteBlue;
  int orangeRed, orangeGreen, orangeBlue;

  // Measure white color
  calibrateColor(255, 255, 255, whiteRed, whiteGreen, whiteBlue);
  //LEDblink();
  delay(1000); // Wait for 1 second to stabilize the readings
  // Measure orange color
  calibrateColor(255, 100, 0, orangeRed, orangeGreen, orangeBlue);

  // Save calibrated values to EEPROM
  //saveCalibrationToEEPROM(whiteRed, whiteGreen, whiteBlue, orangeRed, orangeGreen, orangeBlue); ********* Uncomment this line to save calibration values to EEPROM

  // Done with calibration, now wait for the next step
  Serial.println("Calibration complete. System will perform the task after 2 minutes.");
}

void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue, int samples = 10) {
  // Measure Red intensity (average of multiple samples)
  LEDblink();
  redValue = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, redFilter);
    digitalWrite(S3, greenFilter);
    redValue += pulseIn(OUT, HIGH);
    delay(10); // Small delay between readings
  }
  redValue /= samples; // Average

  // Measure Green intensity
  greenValue = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, greenFilter);
    digitalWrite(S3, redFilter);
    greenValue += pulseIn(OUT, HIGH);
    delay(10);
  }
  greenValue /= samples;

  // Measure Blue intensity
  blueValue = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, blueFilter);
    digitalWrite(S3, redFilter);
    blueValue += pulseIn(OUT, HIGH);
    delay(10);
  }
  blueValue /= samples;
  delay(1000);
  LEDnoblink(); // Turn off LED after calibration
}

void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue) {
  // Save white color values to EEPROM
  EEPROM.write(EEPROM_WHITE_RED, whiteRed);
  EEPROM.write(EEPROM_WHITE_GREEN, whiteGreen);
  EEPROM.write(EEPROM_WHITE_BLUE, whiteBlue);

  // Save orange color values to EEPROM
  EEPROM.write(EEPROM_ORANGE_RED, orangeRed);
  EEPROM.write(EEPROM_ORANGE_GREEN, orangeGreen);
  EEPROM.write(EEPROM_ORANGE_BLUE, orangeBlue);
}

void performTask() {
  // This function will perform the task once calibration is complete and the 2-minute window has passed.
  
  // Retrieve calibrated color values from EEPROM
  int whiteRed = EEPROM.read(EEPROM_WHITE_RED);
  int whiteGreen = EEPROM.read(EEPROM_WHITE_GREEN);
  int whiteBlue = EEPROM.read(EEPROM_WHITE_BLUE);

  int orangeRed = EEPROM.read(EEPROM_ORANGE_RED);
  int orangeGreen = EEPROM.read(EEPROM_ORANGE_GREEN);
  int orangeBlue = EEPROM.read(EEPROM_ORANGE_BLUE);

}


void readRGBArm(int &redVal_arm, int &greenVal_arm, int &blueVal_arm) {
  // Measure Red component
  digitalWrite(S2_arm, LOW);
  digitalWrite(S3_arm, LOW);
  redVal_arm = pulseIn(OUT_arm, LOW);

  // Measure Green component
  digitalWrite(S2_arm, HIGH);
  digitalWrite(S3_arm, HIGH);
  greenVal_arm = pulseIn(OUT_arm, LOW);

  // Measure Blue component
  digitalWrite(S2_arm, LOW);
  digitalWrite(S3_arm, HIGH);
  blueVal_arm = pulseIn(OUT_arm, LOW);
}

String detectColor() {
  // Retrieve calibration values from EEPROM
  int whiteRed = EEPROM.read(EEPROM_WHITE_RED);
  int whiteGreen = EEPROM.read(EEPROM_WHITE_GREEN);
  int whiteBlue = EEPROM.read(EEPROM_WHITE_BLUE);

  int orangeRed = EEPROM.read(EEPROM_ORANGE_RED);
  int orangeGreen = EEPROM.read(EEPROM_ORANGE_GREEN);
  int orangeBlue = EEPROM.read(EEPROM_ORANGE_BLUE);

  // Read current RGB values
  int redVal_arm, greenVal_arm, blueVal_arm;
  readRGBArm(redVal_arm, greenVal_arm, blueVal_arm);

  // Calculate Euclidean distances to white and orange
  int distWhite = sqrt(pow(redVal_arm - whiteRed, 2) + pow(greenVal_arm - whiteGreen, 2) + pow(blueVal_arm - whiteBlue, 2));
  int distOrange = sqrt(pow(redVal_arm - orangeRed, 2) + pow(greenVal_arm - orangeGreen, 2) + pow(blueVal_arm - orangeBlue, 2));

  // Determine the closest color
  if (distWhite < distOrange) {
      return "White";
  } else {
      return "Orange";
  }
}
