#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void moveLeftMotorsForward(int pwmVal);
void moveLeftMotorsBackward(int pwmVal);
void moveRightMotorsForward(int pwmVal);
void moveRightMotorsBackward(int pwmVal);

void moveForward(int distance, int basePWM);  // Updated signature
void moveBackward(int pwmVal);
void moveLeftForward(int pwmVal);
void moveRightForward(int pwmVal);

void rotateLeft(float angle, int basePWM);   // Updated signature
void rotateRight(float angle, int basePWM);  // Updated signature
void moveForwardUntilObstacle();
void moveBackwardDistance(float distance);
void stopMotors();

#endif // MOTOR_CONTROL_H