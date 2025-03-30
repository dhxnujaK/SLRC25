#include <Encoder.h>
#include <PID_v1.h>

// ================== FIR FILTER DEFINITIONS ==================
#define FIR_TAPS 5

// Example 5-tap low-pass FIR coefficients
// Adjust these coefficients as needed.
float firCoeffs[FIR_TAPS] = {0.1, 0.15, 0.5, 0.15, 0.1};

// Buffers to hold recent encoder values for each motor
static float left1Buffer[FIR_TAPS]  = {0, 0, 0, 0, 0};
static float left2Buffer[FIR_TAPS]  = {0, 0, 0, 0, 0};
static float right1Buffer[FIR_TAPS] = {0, 0, 0, 0, 0};
static float right2Buffer[FIR_TAPS] = {0, 0, 0, 0, 0};

// Indices for ring buffers
static int left1Index  = 0;
static int left2Index  = 0;
static int right1Index = 0;
static int right2Index = 0;

// Generic FIR apply function
float applyFIR(float newSample, float* buffer, float* coeffs, int &index) {
  // Store new sample
  buffer[index] = newSample;

  // FIR sum
  float result = 0.0;
  int readPos = index;
  for (int i = 0; i < FIR_TAPS; i++) {
    result += coeffs[i] * buffer[readPos];
    // Move backwards in the ring buffer
    readPos = (readPos - 1 + FIR_TAPS) % FIR_TAPS;
  }

  // Advance index
  index = (index + 1) % FIR_TAPS;

  return result;
}

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
double Kp = 0.4, Ki = 0.0, Kd = 0.3;  // Tune these
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int baseSpeed = 90;  // Base PWM speed (0-255)

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

void moveForward(float distance_cm, int speed = 150) {
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

void moveBackward(float distance_cm, int speed = 150) {
  moveForward(distance_cm, -speed);  // Reuses moveForward with negative speed
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
  setLeftMotor2Speed(0);   // PID-adjusted
  setRightMotor1Speed(baseSpeed);
  setRightMotor2Speed(0);  // PID-adjusted

  while (true) {
    // Read Left2 & Right2 for PID (ignore others)
    noInterrupts();  // Disable interrupts to read encoder values safely
    long newLeft1 = encoderLeft1.read();
    long newLeft2 = encoderLeft2.read();
    long newRight1 = encoderRight1.read();
    long newRight2 = encoderRight2.read();
    interrupts();  // Re-enable interrupts
    
    // Convert to float and apply FIR filter
    float rawLeft1  = (float)newLeft1;
    float rawLeft2  = (float)newLeft2;
    float rawRight1 = (float)newRight1;
    float rawRight2 = (float)newRight2;

    float filteredLeft1  = applyFIR(rawLeft1,  left1Buffer,  firCoeffs, left1Index);
    float filteredLeft2  = applyFIR(rawLeft2,  left2Buffer,  firCoeffs, left2Index);
    float filteredRight1 = applyFIR(rawRight1, right1Buffer, firCoeffs, right1Index);
    float filteredRight2 = applyFIR(rawRight2, right2Buffer, firCoeffs, right2Index);

    long left2  = labs((long)filteredLeft2);
    long right2 = labs((long)filteredRight2);

    // Compute PID correction
    Input = (float)left2 - (float)right2;
    myPID.Compute();

    // Apply correction ONLY to Left2 & Right2
    setLeftMotor2Speed(baseSpeed - Output);
    setRightMotor2Speed(baseSpeed + Output);

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

    //delay(3);  // Prevent CPU overload
  }
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
  myPID.SetOutputLimits(-30, 30);  // Limit correction to ±50 PWM

  Serial.println("Robot Ready: PID Active on Left2/Right2");
}

void loop() {
  // Example: Move forward 1m with PID straightening
  moveForwardStraightPID(100.0);  // Distance in cm
  delay(3000);                    // Pause before repeating
}

// ================== MOVEMENT FUNCTIONS ==================
