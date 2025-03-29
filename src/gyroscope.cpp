#include "gyroscope.h"

MPU6050 mpu;
SimpleKalmanFilter kalmanFilter(2, 2, 0.01); // (Measurement noise, Process noise, Estimation error)
float currentAngle = 0;
unsigned long lastTime = 0;

void initGyro() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized");
    resetGyroAngle();
}

void resetGyroAngle() {
    currentAngle = 0;
    lastTime = millis();
}

float getFilteredAngle() {
    int16_t gyroZ;
    gyroZ = mpu.getRotationZ();
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    float gyroRate = gyroZ / 131.0;
    currentAngle += gyroRate * dt;

    return kalmanFilter.updateEstimate(currentAngle);
}
