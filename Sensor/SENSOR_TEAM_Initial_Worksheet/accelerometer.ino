#include "accelerometer.h"

GY521 accelo(0x68);

float accelThreshold = 0.2;
unsigned long idleTimeout = 30000;
unsigned long lastMovementTime = 0;
bool isMoving = false;

void setupAccelerometer() {
  pinMode(TURN_ON_ACCELEROMETER_PIN, OUTPUT);
  digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW);
  delay(1000);
  accelo.begin();
  
  
  if(accelo.isConnected()) {
    Serial.println("gy521 online");
  }
  else{
    Serial.println("gy521 FAILED");
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
void float_to_string_simpl(float input){
  String tim = String((int)input);
  tim += "|";
  int len = tim.length();
  tim.toCharArray(in_message,100); 
  
}
void print_accelometer_res(){
  Serial.println("<print accelometer>"); 
  //the orientation compared to gravity
  accelo.read();
  float ax = accelo.getAngleX();
  float ay = accelo.getAngleY();
  float az = accelo.getAngleZ();
  //have no idea why this is a feature
  float temp = accelo.getTemperature();
  //the acceleration - change in movement.
  float gx = accelo.getGyroX();
  float gy = accelo.getGyroY();
  float gz = accelo.getGyroZ();
  //print all data - because floats a tad annoying (especially when printing on LCD)
  Serial.println(ax);
  
    Serial.println(az);
      Serial.println(ay);
  Serial.println(gx);
  
    Serial.println(gz);
      Serial.println(gy);
  //new line, because the above calls make no new line.
  Serial.println("");

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
bool isMovingNow() {
  AccelData currentData = readAccelerometerData();
  return checkMovementThreshold(currentData);
}