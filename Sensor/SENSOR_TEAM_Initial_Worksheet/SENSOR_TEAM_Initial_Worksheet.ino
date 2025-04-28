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

void setup_sensors(int bitmask) {
    Wire.begin();
  // Implementation remains the same, but now uses the separate sensor setup functions
  if(bitmask & 4 && (bitmask & 64) == 0) {
    setupAccelerometer();
  }
  if(bitmask & 8) {
    setupPhotoresistor();
  }
  // LCD setup remains here
}

void setup() {
  Serial.begin(115200);
  pinMode(TURN_ON_LCD_PIN, OUTPUT);
  setup_sensors(8 + 4 + 2 + 1);
}

void loop() {
  // Example usage:
  int light = readPhotoresistor();
  Serial.print("Light strength: ");
  Serial.println(light);
  
  monitorAccelerometer();
  print_accelometer_res();
  delay(1000);
}