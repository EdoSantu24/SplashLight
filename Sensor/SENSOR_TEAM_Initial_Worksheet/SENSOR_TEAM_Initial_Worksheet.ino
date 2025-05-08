/*******************************************************
 * File:        SENSOR_TEAM_Initial_Worksheet.ino
 * Description: this file contains the overall frame for our workspace, and our related functions. There are still a few things to clean up, such as all mentions of the LCD.
 * Team:        Sensor Team
 * Created:     2025-04-07
 * Modified:    2025-05-04 (comments)
 *
 * License:     the structure of THIS PARTICULAR COMMENT was made using Chatgpt 
                 (we deemed it to be quite good looking)
                all else in this file has been made by the Sensor Team.
 *******************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "accelerometer.h"
#include "photoresistor.h"

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define TURN_ON_LCD_PIN 13
//21,22
#define SDA_PIN 6
#define SCL_PIN 7

char in_message[100]; // Used to pass message to print between functions
int mode = 0;  // mode = 0 = active, 1 = park, 2 = storage

/**
This function will enable sensors according to the input bitmask
  bit 0 (0x01)1 = I2C (needed for LCD and  accelerometer)
  bit 1 (0x02)2 = LCD
  bit 2 (0x04)4 = Accelerometer
  bit 3 (0x08)8 = photoresistor
  bit 4 (0x10)16 = will assume I2C has already been enabled
  bit 5 (0x20)32 = will assume that LCD has already been enabled 
  bit 6 (0x40)64 = will assume that accelerometer has already been calibrated  
*/
/*### 
This function will enable sensors according to the input bitmask

arguments: 
   int bitmask:
        bit 0 (0x01)1 = I2C (needed for LCD and  accelerometer)
        bit 1 (0x02)2 = LCD
        bit 2 (0x04)4 = Accelerometer
        bit 3 (0x08)8 = photoresistor
        bit 4 (0x10)16 = will assume I2C has already been enabled
        bit 5 (0x20)32 = will assume that LCD has already been enabled 
        bit 6 (0x40)64 = will assume that accelerometer has already been calibrated  
            #3 THIS BITMASK NEEDS TO BE SIMPLIFIED/REDUCED. (remove LCD, and disablement of 

returns: void
#######*/
void setup_sensors(int bitmask) {
    Wire.begin(SDA_PIN,SCL_PIN);
    // Initialize sensors based on bitmask


    //The Accelerometer is ENABLED!!!
    if (bitmask & 4 && (bitmask & 64) == 0) {
        setupAccelerometer();
    }
    //The Accelerometer is DISABLED!!! 
        //#2 (needs to be refitted into the file split)
    if((bitmask & 4) == 0 && bitmask & 64){
        turnOffAccelerometer();
    }



    
    if (bitmask & 8) {
        setupPhotoresistor();
    }
    // LCD setup remains here #!
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Pin mode for LCD #!
  pinMode(TURN_ON_LCD_PIN, OUTPUT);

  // Setup sensors with bitmask (active all sensors for example) #?
  setup_sensors(8 + 4 + 2 + 1);

  // You can also initialize other things here (LCD, etc.) #!
}

void loop() {
if (mode == 0) {  // Active mode
    // Turn on all sensors
        //#3 - Needs to calibrate sensor - assuming it has been powered off will mess with the calibration
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW); //#3
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, LOW);
    
    if (isMovingNow()) {
      // Movement detected, remain in active mode
      reactToMovement();
      print_accelometer_res();   

      int light = readPhotoresistor();
      Serial.print("Light strength: ");
      Serial.println(light);
    } else {
      // No movement detected, switch to parking mode
      mode = 1;
      Serial.println("No movement detected! Switching to parking mode.");
    }
  }
  else if (mode == 1) {  // Park mode
    // Disable photoresistor, keep accelerometer on
      //#3 - Needs to calibrate sensor - assuming it has been powered off will mess with the calibration
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW); //#3
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH);
    
    // Handle accelerometer logic (no need to check photoresistor)
    if (isMovingNow()) {
      // If movement is detected, switch to active mode
      mode = 0;
      Serial.println("Movement detected! Switching to active mode.");
      reactToMovement();
      print_accelometer_res();   

      int light = readPhotoresistor();
      Serial.print("Light strength: ");
      Serial.println(light);
    }
  }
  else if (mode == 2) {  // Storage mode
    // Turn off both accelerometer and photoresistor
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, HIGH); //#4 - might need a call to accelo.reset(); - because the last valid reading seem to remain after power has been shut off to the accelerometer.
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH);
    
    // No need to check accelerometer or photoresistor here
    Serial.println("In storage mode, sensors are off.");
  }
  /*
  if(MESSAGE FROM LORA or button press){
    mode=1; 
    //we go into parking mode when the button is pressed or lora sends message
  }
*/
  delay(1000);  // Delay to simulate a time loop
}
