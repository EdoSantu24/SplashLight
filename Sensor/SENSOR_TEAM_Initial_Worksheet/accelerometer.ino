/*******************************************************
 * File:        accelerometer.ino
 * Description: this file contains all the functions, be it calibration, reading, thresholding, or in/out control signals/flags related to the accelerometer
 * Team:        Sensor Team
 * Created:     2025-04-07
 * Modified:    2025-05-04 (comments)
 *
 * License:     the structure of THIS PARTICULAR COMMENT was made using Chatgpt 
                 (we deemed it to be quite good looking)
                all else in this file has been made by the Sensor Team*
                  *with indirect inspiration from the sensors library and its corresponding creator: https://github.com/RobTillaart/GY521 
 *******************************************************/
//object
#include "accelerometer.h"
GY521 accelo(0x68);

//variables/ control signals/flags
float accelThreshold = 0.2;
unsigned long idleTimeout = 30000;
unsigned long lastMovementTime = 0;
bool isMoving = false;

/*###
This function sets up the accelerometers digital representation, and calibrates it.
Currently it does not have a deactivation equivalent variant and neither will it react appropiately if the accelerometer is not connected (correct behaviour would be to fail, or return "false" - because thats Arduino's convention).

arguments: none

returns: void
#######*/
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


/*###
This function sends a read request to the accelerometer, recieves the reading, stores the data, and returns it

arguments: none

returns: a AccelData struct, which contains the read data.
  - assumes that the Accelerometer is connected... and probably that it has been calibrated.
######*/
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

/*###
This function serves to make it easier to visually decipher the readings of the accelerometer, by turning it into a integer, rather than a decimal 
   -- is supposed to be called for each individual value of the data.
   
arguments: 
  float input: the decimal value to be printed as integer

returns: void
#######*/
void float_to_string_simpl(float input){
  String tim = String((int)input);
  tim += "|";
  int len = tim.length();
  tim.toCharArray(in_message,100); 


/*### TBU
This function takes and prints the result - a deprecated function that could be updated to the |||| - separated format or to utilize the data structure for the accelerometer

arguments: none

returns: void
#######*/
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
/*### TBC
This function ...

arguments: 
  float newThreshhold: 
  float newTimeout:

returns: void
#######*/
void setAccelerometerThresholds(float newThreshhold, float newTimeout) {
  accelThreshold = newThreshhold;
  idleTimeout = newTimeout;
  Serial.print("New thresholds set - Accel: ");
  Serial.print(accelThreshold);
  Serial.print("Idle timeout: ");
  Serial.print(idleTimeout/1000);
  Serial.println("s");
}

/*### TBC
This function ...

arguments: 
   AccelData data:

returns: bool
#######*/
bool checkMovementThreshold(AccelData data) {
  unsigned long previousTime = 0;
  float speedDiff = sqrt(sq(data.ax) + sq(data.ay));
  return speedDiff > accelThreshold;
}

/*### TBC
This function ...

arguments: none

returns: void
#######*/
void reactToMovement() {
  if (!isMoving) {
    isMoving = true;
    lastMovementTime = millis();
    Serial.println("Movement detected!");
  } else {
    lastMovementTime = millis();
  }
}

/*### TBC
This function ...

arguments: none

returns: void
#######*/
void reactToIdle() {
  if (isMoving) {
    isMoving = false;
    unsigned long idleDuration = (millis() - lastMovementTime) / 1000;
    Serial.print("Bike idle for ");
    Serial.print(idleDuration);
    Serial.println(" seconds");
  }
}

/*### TBC
This function ...

arguments: none

returns: void
#######*/
void monitorAccelerometer() {
  AccelData currentData = readAccelerometerData();
  if (checkMovementThreshold(currentData)) {
    reactToMovement();
  } else if (millis() - lastMovementTime > idleTimeout) {
    reactToIdle();
  }
}

/*### TBC
This function ...

arguments: none

returns: bool: 
#######*/
bool isMovingNow() {
  AccelData currentData = readAccelerometerData();
  return checkMovementThreshold(currentData);
}
