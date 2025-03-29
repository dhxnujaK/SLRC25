#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Wire.h>
#include <MPU6050.h>
#include "SimpleKalmanFilter.h"

void initGyro();
float getFilteredAngle();
void resetGyroAngle();

#endif