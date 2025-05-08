#include <TinyGPSPlus.h>
#include <QuickDebug.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "accelerometer.h"
#include "photoresistor.h"

#include <HardwareSerial.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define TURN_ON_LCD_PIN 13
//21,22
#define SDA_PIN 6
#define SCL_PIN 7

// GPS setup on UART2 using GPIO 6 (RX) and GPIO 7 (TX)
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2

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
    Wire.begin();
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
  Serial.begin(115200);
  pinMode(TURN_ON_LCD_PIN, OUTPUT);
  
  // Setup GPS UART (pin 6 = RX, 7 = TX)
  gpsSerial.begin(9600, SERIAL_8N1, 6, 7);

  setup_sensors(8 + 4 + 2 + 1);
}

void loop() {
  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Print GPS location if available
  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  }

  if (mode == 0) {  // Active mode
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW);
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, LOW);
    
    if (isMovingNow()) {
      reactToMovement();
      print_accelometer_res();   

      int light = readPhotoresistor();
      Serial.print("Light strength: ");
      Serial.println(light);
    } else {
      mode = 1;
      Serial.println("No movement detected! Switching to parking mode.");
    }
  }
  else if (mode == 1) {  // Park mode
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW);
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH);
    
    if (isMovingNow()) {
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
    digitalWrite(TURN_ON_ACCELEROMETER_PIN, HIGH);
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH);
    Serial.println("In storage mode, sensors are off.");
  }

  delay(1000);
}
