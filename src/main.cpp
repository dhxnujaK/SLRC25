#include <Wire.h>
#include <Adafruit_PCA9685.h>

// Create an instance of the PCA9685 object
Adafruit_PCA9685 pwm = Adafruit_PCA9685();

void setup() {
  Serial.begin(9600);
  
  // Initialize the PCA9685 module
  pwm.begin();

  // Set the PWM frequency to 60 Hz (standard for servos)
  pwm.setPWMFreq(60);

  // Let the user know the program is starting
  Serial.println("PCA9685 Servo Control Initialized");
}

void loop() {
  // Control servo on channel 0 from 0 degrees to 180 degrees
  for (int pos = 0; pos < 180; pos++) {
    // Map the position (0-180) to PWM values (0-4095)
    pwm.writeMicroseconds(0, map(pos, 0, 180, 500, 2500));  // Adjust PWM for servo range
    delay(15);  // Wait for the servo to reach the position
  }

  // Control the servo on channel 0 from 180 degrees back to 0 degrees
  for (int pos = 180; pos >= 0; pos--) {
    pwm.writeMicroseconds(0, map(pos, 0, 180, 500, 2500));  // Adjust PWM for servo range
    delay(15);  // Wait for the servo to reach the position
  }

  // Control other servos similarly by changing the channel number
  // Example for controlling servo 1 (using channel 1):
  for (int pos = 0; pos < 180; pos++) {
    pwm.writeMicroseconds(1, map(pos, 0, 180, 500, 2500));  // Adjust PWM for servo range
    delay(15);  // Wait for the servo to reach the position
  }

  for (int pos = 180; pos >= 0; pos--) {
    pwm.writeMicroseconds(1, map(pos, 0, 180, 500, 2500));  // Adjust PWM for servo range
    delay(15);  // Wait for the servo to reach the position
  }
}