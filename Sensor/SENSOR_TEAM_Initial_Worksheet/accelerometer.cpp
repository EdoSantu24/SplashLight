#include "accelerometer.h"

GY521 accelo(0x68);

float accelThreshold = 0.2;
unsigned long idleTimeout = 30000;
unsigned long lastMovementTime = 0;
bool isMoving = false;

void setupAccelerometer() {
  digitalWrite(TURN_ON_ACCELEROMETER_PIN, HIGH);
  accelo.begin();
  
  if(accelo.isConnected()) {
    Serial.println("gy521 online");
  }
  
  accelo.calibrate(100, 0, 0, false);
  accelo.setAccelSensitivity(0);
  accelo.setGyroSensitivity(0);
  accelo.setNormalize(false);
  accelo.setThrottle();
}

AccelData readAccelerometerData() {
  AccelData data;
  accelo.read();
  
  data.ax = accelo.getAccelX();
  data.ay = accelo.getAccelY();
  data.az = accelo.getAccelZ();
  
  data.gx = accelo.getGyroX();
  data.gy = accelo.getGyroY();
  data.gz = accelo.getGyroZ();
  
  data.angleX = accelo.getAngleX();
  data.angleY = accelo.getAngleY();
  data.angleZ = accelo.getAngleZ();
  
  return data;
}

void setAccelerometerThresholds(float newThreshhold, float newTimeout) {
  accelThreshold = newThreshhold;
  idleTimeout = newTimeout;
  Serial.print("New thresholds set - Accel: ");
  Serial.print(accelThreshold);
  Serial.print("Idle timeout: ");
  Serial.print(idleTimeout/1000);
  Serial.println("s");
}

bool checkMovementThreshold(AccelData data) {
  unsigned long previousTime = 0;
  float speedDiff = sqrt(sq(data.ax) + sq(data.ay));
  return speedDiff > accelThreshold;
}

void reactToMovement() {
  if (!isMoving) {
    isMoving = true;
    lastMovementTime = millis();
    Serial.println("Movement detected!");
  } else {
    lastMovementTime = millis();
  }
}

void reactToIdle() {
  if (isMoving) {
    isMoving = false;
    unsigned long idleDuration = (millis() - lastMovementTime) / 1000;
    Serial.print("Bike idle for ");
    Serial.print(idleDuration);
    Serial.println(" seconds");
  }
}

void monitorAccelerometer() {
  AccelData currentData = readAccelerometerData();
  
  if (checkMovementThreshold(currentData)) {
    reactToMovement();
  } else if (millis() - lastMovementTime > idleTimeout) {
    reactToIdle();
  }
}