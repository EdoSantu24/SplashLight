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
float accelThreshold = 0.2; //#0
unsigned long idleTimeout = 30000;
unsigned long lastMovementTime = 0;
bool isMoving = false;
bool p_unable = false;
bool isMeantToBeOn = true;
char in_message2[100]; // Used to pass message to print between functions
/*###
This function sets up the accelerometers digital representation, and calibrates it.
  #2 Currently it does not have a deactivation equivalent variant and neither will it react appropiately if the accelerometer is not connected (correct behaviour would be to fail, or return "false" - because thats Arduino's convention).

arguments: none

returns: void
#######*/
void setupAccelerometer() {
  p_unable = false;
  isMeantToBeOn = true;
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
This function sets up the accelerometers digital representation, and calibrates it.
  #2 Currently it does not have a deactivation equivalent variant and neither will it react appropiately if the accelerometer is not connected (correct behaviour would be to fail, or return "false" - because thats Arduino's convention).

arguments: none

returns: void
#######*/
void turnOffAccelerometer() {
    p_unable = false;
    isMeantToBeOn = false;
    pinMode(TURN_ON_ACCELEROMETER_PIN, OUTPUT);
    //accelo.end(); //not a function
    //Serial.println("AAAAAAAAAAAAAAAAAAHHHHHHHHHH");
    accelo.reset();
    delay(10);
    digitalWrite(TURN_ON_ACCELEROMETER_PIN,HIGH);
}


/*###
This function sends a read request to the accelerometer, recieves the reading, stores the data, and returns it

arguments: none

returns: a AccelData struct, which contains the read data.
  - assumes that the Accelerometer is connected... and probably that it has been calibrated.
######*/
AccelData readAccelerometerData() {
  AccelData data;
  if(accelo.isConnected()) {
    
    
    if(p_unable){
      if(isMeantToBeOn){
        Serial.println("REBOOT gy521");
        setupAccelerometer();
      } else {
        Serial.println("UNAUTHORIZED CALL TO READ - THE ACCELEROMETER IS NOT MEANT TO BE ON\n!\n!\n!\n!\n!\n!");
      }
    }
  }
  else{
    
    if(isMeantToBeOn){
      Serial.println("gy521 FAILED");
      accelo.reset();
      p_unable = true;
    } else {
      Serial.println("UNAUTHORIZED CALL TO READ - THE ACCELEROMETER IS NOT MEANT TO BE ON\n!\n!\n!\n!\n!\n!");
    }
    
  }
  
  
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
  tim.toCharArray(in_message2,100); 


/*### 
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
      Serial.println(sqrt(sq(ax) + sq(ay) + sq(az)));
  //new line, because the above calls make no new line.
  Serial.println("");

}
/*### 
This function sets a new threshold and timeout. threshold is how much acceleration measured is needed to reconfirm that the device (and bike) is moving. timeout is time in between confirmed movements, before it considers itself to be stationary.
  #0 - might need an upper and lower threshold (so it do not switch constantly between STATIONARY and MOVING)


arguments: 
  float newThreshhold: the higher, the less sensitive it is
  float newTimeout: the higher, the more time it has without movement before reacting to its idleness. it is in ms.

returns: void
#######*/
void setAccelerometerThresholds(float newThreshhold, float newTimeout) {
  accelThreshold = newThreshhold; //#0
  idleTimeout = newTimeout;
  Serial.print("New thresholds set - Accel: ");
  Serial.print(accelThreshold);//#0
  Serial.print("Idle timeout: ");
  Serial.print(idleTimeout/1000);
  Serial.println("s");
}

/*###
This function checks if threshold has been exceeded. if so, it resets its idle timer.
  #0 - might need an upper and lower threshold (so it do not switch constantly between STATIONARY and MOVING)
arguments: 
   AccelData data: the reading provided to this function - it needs both x and y to determine acceleration...
     #5 - should it not also use z coordinates? in case whe havent placed the accelerometer in a perfectly plane orientation?

returns: a bool which tells if the acceleration indicates movement currently.
#######*/
bool checkMovementThreshold(AccelData data) {
  float speedDiff = sqrt(sq(data.gx) + sq(data.gy) + sq(data.gz)); // Now includes Z-axis
  //Serial.println(accelThreshold);
  //Serial.println(sqrt(sq(data.gx) + sq(data.gy) + sq(data.gz)));
  //Serial.println(speedDiff > accelThreshold);
  return speedDiff > accelThreshold;
}


/*### 
This function will, if movement has been detected, reset the idle timer to the current time and it will set the appropiate flag to high (isMoving)
  //#6 - Why do we do an if else statement where for both cases, we say: "lastMovementTime = millis();"? would it be more correct to only do this, if we detect movement? 
    // i know we have the "isMovingNow()" function - but if we just modify this a tad, then it would do the same thing for us.
arguments: none

returns: void
#######*/
void reactToMovement() {
  if (!isMoving) {
    isMoving = true;
    lastMovementTime = millis();
    Serial.println("Movement detected!");
  } else {
    lastMovementTime = millis(); //#6
  }
}

/*### 
This function tallies one final time how long the device (and bike) has remained stationary - which is due to the fact, that we are not interested in how long it has remained idle. though actually, this could be an excellent indicator of when a child was kidnapped - by thus giving us the lower bound timeframe for the kidnapping.... though this supposed feature is beyond the scope of this course

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

/*###
This function is not in use currently.
  - its purpose was to verify that the accelerometer thresholding functions worked as intended.

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

/*### 
This function checks only whenever the bike is moving currently - by comparing its accelerations to the threshold.

arguments: none

returns: a bool which is HIGH if it is moving and FALSE if not.  
#######*/
bool isMovingNow() {
  AccelData currentData = readAccelerometerData();
  return checkMovementThreshold(currentData);
}
