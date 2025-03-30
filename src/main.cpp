#include <Wire.h>
#include <MPU6050_light.h>

// Create MPU6050 instance
MPU6050 mpu(Wire);

unsigned long timer = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Initializing MPU6050...");
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("MPU6050 failed with error code: ");
        Serial.println(status);
        while (1); // Halt if initialization fails
    }

    Serial.println("Calibrating MPU6050...");
    delay(1000);
    mpu.calcOffsets();  // Automatically calibrate accelerometer & gyroscope
    Serial.println("Calibration Complete!");
}

void loop() {
    mpu.update();

    // Print Yaw, Pitch, and Roll values
    if (millis() - timer > 100) { // Print every 100ms
        Serial.print("Yaw: ");
        Serial.print(mpu.getAngleZ(), 2);  // Yaw
        Serial.print(" | Pitch: ");
        Serial.print(mpu.getAngleX(), 2);  // Pitch
        Serial.print(" | Roll: ");
        Serial.println(mpu.getAngleY(), 2);  // Roll
        timer = millis();
    }
}
