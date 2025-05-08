#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


#include "GY521.h"

// Accelerometer pin
#define TURN_ON_ACCELEROMETER_PIN 18

// Accelerometer values:
extern float accelThreshold;  // in m/s²
extern unsigned long idleTimeout; // 30 sec timeout 
extern unsigned long lastMovementTime;
extern bool isMoving;

struct AccelData {
  float ax, ay, az;  // Acceleration (m/s²)
  float gx, gy, gz;  // Gyro (deg/s)
  float angleX, angleY, angleZ; // Angles
};

extern GY521 accelo;

void setupAccelerometer();
AccelData readAccelerometerData();
void setAccelerometerThresholds(float newThreshhold, float newTimeout);
bool checkMovementThreshold(AccelData data);
void reactToMovement();
void reactToIdle();
void monitorAccelerometer();
void print_accelometer_res();
bool isMovingNow();
#endif
