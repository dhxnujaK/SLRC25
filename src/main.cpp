#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define LED 13

#define XSHUT_PIN 41
#define S0 46
#define S1 47
#define S2 42
#define S3 44
#define OUT 43

// Arm Color Sensor
#define ARM_S0 28
#define ARM_S1 26
#define ARM_S2 35
#define ARM_S3 34
#define ARM_OUT 29

#define WHITE_ADDR 0       // EEPROM address for white color (0-5)
#define YELLOW_ADDR 10     // EEPROM address for yellow color (10-15)

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

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- Function to Set Motor Speeds ---
void moveForward(int speed, float distance);
void readRGB();
void grab_ball();

void readColor(int &r, int &g, int &b) {
  digitalWrite(ARM_S2, LOW); digitalWrite(ARM_S3, LOW);   // RED
  r = pulseIn(ARM_OUT, LOW);
  delay(50);

  digitalWrite(ARM_S2, HIGH); digitalWrite(ARM_S3, HIGH); // GREEN
  g = pulseIn(ARM_OUT, LOW);
  delay(50);

  digitalWrite(ARM_S2, LOW); digitalWrite(ARM_S3, HIGH);  // BLUE
  b = pulseIn(ARM_OUT, LOW);
  delay(50);
}
void calibrateColor(String color) {
  digitalWrite(LED, HIGH);
  delay(1000);
  int r, g, b;
  Serial.println("Place the " + color + " ball under the sensor and press a key...");
  while (!Serial.available()); Serial.read();  // wait for key

  readColor(r, g, b);
  Serial.println(color + " RGB: " + String(r) + ", " + String(g) + ", " + String(b));

  int addr = (color == "white") ? WHITE_ADDR : YELLOW_ADDR;
  EEPROM.put(addr, r);
  EEPROM.put(addr + 2, g);
  EEPROM.put(addr + 4, b);

  Serial.println(color + " color data saved to EEPROM!");
  digitalWrite(LED,LOW);
}
void readColorFromEEPROM(int addr, int &r, int &g, int &b) {
  EEPROM.get(addr, r);
  EEPROM.get(addr + 2, g);
  EEPROM.get(addr + 4, b);
}
String identifyColor() {
  int rNow, gNow, bNow;
  readColor(rNow, gNow, bNow);

  int rW, gW, bW, rY, gY, bY;
  readColorFromEEPROM(WHITE_ADDR, rW, gW, bW);
  readColorFromEEPROM(YELLOW_ADDR, rY, gY, bY);

  long distWhite = sq(rNow - rW) + sq(gNow - gW) + sq(bNow - bW);
  long distYellow = sq(rNow - rY) + sq(gNow - gY) + sq(bNow - bY);

  return (distWhite < distYellow) ? "white" : "yellow";
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
      //moveForward(90, 1);
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
  moveServoSmoothly(GRIPPER_SERVO,68); delay(1000);

  String color = identifyColor();
  if(color == "'white"){
    moveServoSmoothly(BASE_SERVO,100);
  }
  delay(500);

  moveServoSmoothly(ARM_SERVO, 117);
  moveServoSmoothlyUpper(WRIST_SERVO, 0);
  moveServoSmoothly(GRIPPER_SERVO, 98); delay(500);
  
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

  pinMode(LED, OUTPUT);

  // Arm color sensor
  pinMode(ARM_S0, OUTPUT); pinMode(ARM_S1, OUTPUT);
  pinMode(ARM_S2, OUTPUT); pinMode(ARM_S3, OUTPUT);
  pinMode(ARM_OUT, INPUT);
  
  digitalWrite(ARM_S0, HIGH);  // 20% scaling
  digitalWrite(ARM_S1, LOW);

  /* for(int i=0; i<5; i++){
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
  calibrateColor("white");
  delay(1000);
  calibrateColor("yellow");
  delay(1000); */

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

    // Initialize all servos to a neutral position
    moveServoSmoothly(BASE_SERVO, 90); delay(1000);
    moveServoSmoothly(ARM_SERVO, 116); delay(1000);
    moveServoSmoothlyUpper(WRIST_SERVO, 5); delay(1000);
    moveServoSmoothly(GRIPPER_SERVO, 98); delay(1000);

    Serial.println("Base servo moved to 90 degrees.");

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

  turnLeft(150,90);
  delay(500);
  moveForward(90,30);
  delay(500);
  turnRight(150,90);
  delay(500);
  moveUntillGreen(90); // Move until green is detected

  delay(1000);
}
