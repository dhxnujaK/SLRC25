#include <Encoder.h>
#include <PID_v1.h>

// --- Encoders (All 4) ---
Encoder encoderLeft1(34, 35);   // Left1
Encoder encoderLeft2(19, 18);   // Left2 (PID-controlled)
Encoder encoderRight1(26, 27);  // Right1
Encoder encoderRight2(2, 3);    // Right2 (PID-controlled)

// --- Motor Pins (All 4) ---
// Left Motor 1
const int leftMotor1PWMPin = 10;
const int leftMotor1DirPin1 = 31;
const int leftMotor1DirPin2 = 30;

// Left Motor 2 (PID-controlled)
const int leftMotor2PWMPin = 9;
const int leftMotor2DirPin1 = 32;
const int leftMotor2DirPin2 = 33;

// Right Motor 1
const int rightMotor1PWMPin = 12;
const int rightMotor1DirPin1 = 38;
const int rightMotor1DirPin2 = 39;

// Right Motor 2 (PID-controlled)
const int rightMotor2PWMPin = 11;
const int rightMotor2DirPin1 = 22;
const int rightMotor2DirPin2 = 23;

// --- Enable Pins ---
const int EnablePin1 = 25;  // Left motors
const int EnablePin2 = 24;  // Right motors

// --- Robot Parameters ---
const float WHEEL_DIAMETER_CM = 4.5;      // Measure your wheel
const int COUNTS_PER_REVOLUTION = 100;    // From encoder specs
const float GEAR_RATIO = 15;              // Gear reduction ratio
const float DISTANCE_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / (COUNTS_PER_REVOLUTION * GEAR_RATIO);

// --- PID Control (Left2 & Right2 Only) ---
double Setpoint, Input, Output;         // PID variables
double Kp = 0.2, Ki = 0.03, Kd = 0.4;  // Tune these
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int baseSpeed = 100;  // Base PWM speed (0-255)

// ================== MOVEMENT FUNCTIONS ==================
// ================== MOTOR CONTROL ==================
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

// ================== UTILITY FUNCTIONS ==================
void stopAllMotors() {
  setLeftMotor1Speed(0);
  setLeftMotor2Speed(0);
  setRightMotor1Speed(0);
  setRightMotor2Speed(0);
  Serial.println("Motors Stopped");
}
void moveForwardStraightPID(float distance_cm) {
  // Reset ALL encoders
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);

  long targetCounts = distance_cm / DISTANCE_PER_COUNT;

  // Start ALL motors at base speed
  setLeftMotor1Speed(baseSpeed);
  setLeftMotor2Speed(baseSpeed);   // PID-adjusted
  setRightMotor1Speed(baseSpeed);
  setRightMotor2Speed(baseSpeed);  // PID-adjusted

  while (true) {
    // Read Left2 & Right2 for PID (ignore others)
    long left2 = abs(encoderLeft2.read());
    long right2 = abs(encoderRight2.read());

    // Compute PID correction
    Input = left2 - right2;
    myPID.Compute();

    // Apply correction ONLY to Left2 & Right2
    setLeftMotor2Speed(baseSpeed + Output);  // Slow down if ahead
    setRightMotor2Speed(baseSpeed*1.07 - Output); // Speed up if behind

    // Debug output
    Serial.print("L2: ");
    Serial.print(left2);
    Serial.print(" | R2: ");
    Serial.print(right2);
    Serial.print(" | PID Out: ");
    Serial.println(Output);

    // Stop when any motor reaches target
    if (left2 >= targetCounts || right2 >= targetCounts) {
      stopAllMotors();
      break;
    }

    delay(10);  // Prevent CPU overload
  }
}



void moveForward(float distance_cm, int speed = 70) {
  // Reset encoders
  encoderLeft1.write(0);
  encoderLeft2.write(0);
  encoderRight1.write(0);
  encoderRight2.write(0);

  long target = distance_cm / DISTANCE_PER_COUNT;

  // Start all motors
  setLeftMotor1Speed(speed);
  setLeftMotor2Speed(speed);
  setRightMotor1Speed(speed);
  setRightMotor2Speed(speed);

  // Wait until target reached
  while (abs(encoderLeft2.read()) < target && abs(encoderRight2.read()) < target) {
    delay(10);
  }

  stopAllMotors();
}

void moveBackward(float distance_cm, int speed = 70) {
  moveForward(distance_cm, -speed);  // Reuses moveForward with negative speed
}

// ====================== CORE FUNCTIONS ======================
void setup() {
  Serial.begin(115200);

  // Initialize ALL motor pins
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

  // Configure PID
  Setpoint = 0;  // Target: Zero difference between Left2/Right2
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-50, 50);  // Limit correction to ±50 PWM

  Serial.println("Robot Ready: PID Active on Left2/Right2");
}

void loop() {
  // Example: Move forward 1m with PID straightening
  moveForwardStraightPID(100.0);  // Distance in cm
  delay(3000);                    // Pause before repeating
}

