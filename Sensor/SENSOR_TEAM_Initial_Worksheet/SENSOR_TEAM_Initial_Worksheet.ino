#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "accelerometer.h"
#include "photoresistor.h"

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define TURN_ON_LCD_PIN 13

#define SDA_PIN 21
#define SCL_PIN 22

char in_message[100]; // Used to pass message to print between functions
int mode = 0;  // mode = 0 = active, 1 = park, 2 = storage

void setup_sensors(int bitmask) {
    Wire.begin();
    // Initialize sensors based on bitmask
    if (bitmask & 4 && (bitmask & 64) == 0) {
        setupAccelerometer();
    }
    if (bitmask & 8) {
        setupPhotoresistor();
    }
    // LCD setup remains here
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Pin mode for LCD
  pinMode(TURN_ON_LCD_PIN, OUTPUT);

  // Setup sensors with bitmask (active all sensors for example)
  setup_sensors(8 + 4 + 2 + 1);

  // You can also initialize other things here (LCD, etc.)
}

void loop() {
  if (mode == 0) {  // Active mode
    // Turn on all sensors
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW);
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, LOW);
    
    if (isMovingNow()) {
      reactToMovement();
      print_accelometer_res();   

      int light = readPhotoresistor();
      Serial.print("Light strength: ");
      Serial.println(light);
    }
  }
  else if (mode == 1) {  // Park mode
    // Disable photoresistor, keep accelerometer on
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW);
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
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, HIGH);
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
