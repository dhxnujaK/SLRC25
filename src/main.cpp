#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PWM driver at the default I2C address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVO_FREQ 50  // Frequency for analog servos (Hz)
#define SERVOMIN  150  // Minimum pulse length count (out of 4096)
#define SERVOMAX  600  // Maximum pulse length count (out of 4096)

// Servo channels for robot arm joints
#define BASE_SERVO      0
#define SHOULDER_SERVO  1
#define ELBOW_SERVO     2
#define WRIST_SERVO     3
#define GRIPPER_SERVO   4

// Function to move a servo to a position (0-180 degrees mapped to pulse length)
void moveServo(uint8_t servoNum, uint8_t angle) {
  // Map angle (0-180) to pulse length (SERVOMIN-SERVOMAX)
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulse);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Robot Arm Servos...");

  Wire.begin();  // Start I2C
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize all servos to a neutral position
  moveServo(BASE_SERVO, 90);
  moveServo(SHOULDER_SERVO, 90);
  moveServo(ELBOW_SERVO, 90);
  moveServo(WRIST_SERVO, 90);
  moveServo(GRIPPER_SERVO, 90);
}

void loop() {
  // Example sequence: simple movements for each joint

  moveServo(BASE_SERVO, 45); delay(500);
  moveServo(SHOULDER_SERVO, 120); delay(500);
  moveServo(ELBOW_SERVO, 60); delay(500);
  moveServo(WRIST_SERVO, 100); delay(500);
  moveServo(GRIPPER_SERVO, 30); delay(1000);  // Close gripper

  moveServo(GRIPPER_SERVO, 90); delay(1000);  // Open gripper
  moveServo(BASE_SERVO, 90); delay(500);
  moveServo(SHOULDER_SERVO, 90); delay(500);
  moveServo(ELBOW_SERVO, 90); delay(500);
  moveServo(WRIST_SERVO, 90); delay(500);
}