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
#define ARM_SERVO       1
#define WRIST_SERVO     2
#define GRIPPER_SERVO   3

#define DEFAULT_SPEED 20  // Speed range: 1 (slow) to 100 (fast)

int int_angles[4] = {90,90,90,90};
int drop_angles[4] = {90,110,0,98};
int current_angles[4] = {90,90,90,90};

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

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Robot Arm Servos...");

  Wire.begin();  // Start I2C
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize all servos to a neutral position
  /* moveServo(BASE_SERVO, 90); delay(500);
  moveServo(ARM_SERVO, 110); delay(500);
  moveServo(WRIST_SERVO, 0); delay(500); // lower values for upward position
  moveServo(GRIPPER_SERVO, 98); delay(500); // 98 for open, 68 for closed */

  moveServoSmoothly(BASE_SERVO, 90);
  moveServoSmoothly(ARM_SERVO, 120);
  

}
bool run = true;
void loop() {
  // Example sequence: simple movements for each joint

  if(!run) {
    //moveServo(BASE_SERVO, 45); delay(500);
    //moveServo(SHOULDER_SERVO, 120); delay(500);
    //moveServo(ELBOW_SERVO, 60); delay(500);
    moveServo(WRIST_SERVO, 100); delay(500);
    moveServo(GRIPPER_SERVO, 98); delay(1000);  // Close gripper
  
    moveServo(GRIPPER_SERVO, 130); delay(1000);  // Open gripper
    //moveServo(BASE_SERVO, 90); delay(500);
    moveServo(WRIST_SERVO, 90); delay(500);
  } else {
    return;  // Prevents re-running the sequence
  }

}



/* moveServo(BASE_SERVO, 90); delay(500);
moveServo(ARM_SERVO, 110); delay(500);
moveServo(WRIST_SERVO, 0); delay(500); // lower values for upward position
moveServo(GRIPPER_SERVO, 98); delay(500); // 98 for open, 68 for closed */   // values for drop ball