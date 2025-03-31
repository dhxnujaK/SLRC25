#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include <EEPROM.h>

// --- Pin Definitions ---
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define OUT 8
#define BUZZER 13

// --- EEPROM Addresses ---
#define EEPROM_WHITE_RED 0
#define EEPROM_WHITE_GREEN 1
#define EEPROM_WHITE_BLUE 2
#define EEPROM_ORANGE_RED 3
#define EEPROM_ORANGE_GREEN 4
#define EEPROM_ORANGE_BLUE 5
#define CALIBRATION_TIME_LIMIT 120000

// --- Global Variables ---
unsigned long calibrationStartTime;
MPU6050 mpu(Wire);

// --- Encoders ---
Encoder encoderLeft1(34, 35);
Encoder encoderLeft2(19, 18);
Encoder encoderRight1(26, 27);
Encoder encoderRight2(2, 3);

// --- Motor Pins ---
const int leftMotor1PWMPin = 10, leftMotor1DirPin1 = 31, leftMotor1DirPin2 = 30;
const int leftMotor2PWMPin = 9, leftMotor2DirPin1 = 32, leftMotor2DirPin2 = 33;
const int rightMotor1PWMPin = 12, rightMotor1DirPin1 = 38, rightMotor1DirPin2 = 39;
const int rightMotor2PWMPin = 11, rightMotor2DirPin1 = 22, rightMotor2DirPin2 = 23;
const int EnablePin1 = 25, EnablePin2 = 24;

// --- Robot Parameters ---
const float WHEEL_DIAMETER_CM = 5.4;
const int COUNTS_PER_REV = 100;
const float GEAR_RATIO = 15;
const float DISTANCE_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / (COUNTS_PER_REV * GEAR_RATIO);

// --- PID Constants ---
const float KP = 2.0;
const float KI = 0.1;
const float KD = 0.05;
float lastError = 0;
float integral = 0;

// --- Function Declarations ---
void startCalibration();
void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue);
void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue);
int readColor(int redFilter, int greenFilter, int blueFilter);
void performColorCheck();
void setMotorSpeed(int left1, int left2, int right1, int right2);
void stopAllMotors();
void moveDistance(float targetDistance, int baseSpeed);
void turnLeft(int speed, float targetAngle);
void turnRight(int speed, float targetAngle);
float calculatePID(float targetDistance, float currentDistance);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize Color Sensor
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(OUT, INPUT);
    pinMode(BUZZER, OUTPUT);
    digitalWrite(S0, HIGH);
    digitalWrite(S1, HIGH);

    // Initialize MPU6050
    if (mpu.begin() != 0) {
        Serial.println("MPU6050 failed!");
        while (1);
    }
    mpu.calcOffsets();

    // Initialize Motors
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

    startCalibration();
}

void loop() {
    if (millis() - calibrationStartTime > CALIBRATION_TIME_LIMIT) {
        // Movement sequence with color checks
        moveDistance(30, 90);
        performColorCheck();
        delay(500);
        
        turnLeft(120, 90);
        performColorCheck();
        delay(500);
        
        moveDistance(20, 90);
        performColorCheck();
        delay(500);
        
        turnRight(120, 90);
        performColorCheck();
        delay(500);
        
        stopAllMotors();
        delay(1000);
    }
}

float calculatePID(float targetDistance, float currentDistance) {
    float error = targetDistance - currentDistance;
    integral += error;
    float derivative = error - lastError;
    lastError = error;
    return (KP * error) + (KI * integral) + (KD * derivative);
}

void setMotorSpeed(int left1, int left2, int right1, int right2) {
    analogWrite(leftMotor1PWMPin, abs(left1 * 0.92));
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

void moveDistance(float targetDistance, int baseSpeed) {
    encoderLeft2.write(0);
    encoderRight2.write(0);
    integral = 0;
    lastError = 0;
    
    while(true) {
        long encoderCount = (abs(encoderLeft2.read()) + abs(encoderRight2.read())) / 2;
        float currentDistance = encoderCount * DISTANCE_PER_COUNT;
        float pidOutput = calculatePID(targetDistance, currentDistance);
        int adjustedSpeed = constrain(baseSpeed + pidOutput, -255, 255);
        
        setMotorSpeed(adjustedSpeed, adjustedSpeed, adjustedSpeed, adjustedSpeed);
        
        if(abs(targetDistance - currentDistance) < 0.5) {
            stopAllMotors();
            break;
        }
        delay(10);
    }
}

void stopAllMotors() {
    setMotorSpeed(0, 0, 0, 0);
}

void turnLeft(int speed, float targetAngle) {
    mpu.update();
    float startYaw = mpu.getAngleZ();
    float targetYaw = startYaw + targetAngle;
    
    setMotorSpeed(-speed, -speed, speed, speed);
    
    while (true) {
        mpu.update();
        if (mpu.getAngleZ() >= targetYaw) {
            stopAllMotors();
            break;
        }
        delay(10);
    }
}

void turnRight(int speed, float targetAngle) {
    mpu.update();
    float startYaw = mpu.getAngleZ();
    float targetYaw = startYaw - targetAngle;
    
    setMotorSpeed(speed, speed, -speed, -speed);
    
    while (true) {
        mpu.update();
        if (mpu.getAngleZ() <= targetYaw) {
            stopAllMotors();
            break;
        }
        delay(10);
    }
}

// Color Sensor Functions
void startCalibration() {
    tone(BUZZER, 1000);
    delay(200);
    noTone(BUZZER);

    calibrationStartTime = millis();
    int whiteRed, whiteGreen, whiteBlue;
    int orangeRed, orangeGreen, orangeBlue;

    calibrateColor(HIGH, HIGH, LOW, whiteRed, whiteGreen, whiteBlue);
    calibrateColor(LOW, HIGH, LOW, orangeRed, orangeGreen, orangeBlue);
    saveCalibrationToEEPROM(whiteRed, whiteGreen, whiteBlue, orangeRed, orangeGreen, orangeBlue);

    tone(BUZZER, 1500);
    delay(200);
    noTone(BUZZER);
}

void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue) {
    digitalWrite(S2, redFilter);
    digitalWrite(S3, greenFilter);
    redValue = pulseIn(OUT, HIGH);
    Serial.print("Red Value: ");
    Serial.println(redValue);

    digitalWrite(S2, greenFilter);
    digitalWrite(S3, redFilter);
    greenValue = pulseIn(OUT, HIGH);
    Serial.print("Green Value: ");
    Serial.println(greenValue);

    digitalWrite(S2, blueFilter);
    digitalWrite(S3, redFilter);
    blueValue = pulseIn(OUT, HIGH);
    Serial.print("Blue Value: ");
    Serial.println(blueValue);
}

void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue) {
    EEPROM.write(EEPROM_WHITE_RED, whiteRed);
    EEPROM.write(EEPROM_WHITE_GREEN, whiteGreen);
    EEPROM.write(EEPROM_WHITE_BLUE, whiteBlue);
    EEPROM.write(EEPROM_ORANGE_RED, orangeRed);
    EEPROM.write(EEPROM_ORANGE_GREEN, orangeGreen);
    EEPROM.write(EEPROM_ORANGE_BLUE, orangeBlue);
}

int readColor(int redFilter, int greenFilter, int blueFilter) {
    digitalWrite(S2, redFilter);
    digitalWrite(S3, greenFilter);
    int red = pulseIn(OUT, HIGH);
    
    digitalWrite(S2, greenFilter);
    digitalWrite(S3, redFilter);
    int green = pulseIn(OUT, HIGH);
    
    digitalWrite(S2, blueFilter);
    digitalWrite(S3, redFilter);
    int blue = pulseIn(OUT, HIGH);
    
    return red + green + blue;
}

void performColorCheck() {
    int colorSum = readColor(HIGH, HIGH, LOW);
    if (colorSum > 1000) {
        Serial.println("White Detected!");
        tone(BUZZER, 2000, 200);
    }
}