#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>

#define XSHUT_PIN 41
#define S0 46
#define S1 47
#define S2 42
#define S3 44
#define OUT 43

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

// Variables to store pulse width measurements
int redValue, greenValue, blueValue;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- Function to Set Motor Speeds ---
void moveForward(int speed, float distance);
void readRGB();

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
  targetAngle += 58.13559;
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
  targetAngle += 58.13559;
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

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Set frequency scaling to 20% (recommended for better resolution)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  digitalWrite(EnablePin1, HIGH);
  digitalWrite(EnablePin2, HIGH);
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
