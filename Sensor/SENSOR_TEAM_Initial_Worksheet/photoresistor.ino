#include "photoresistor.h"
int lightThreshold = 500;
void setupPhotoresistor() {
  pinMode(TURN_ON_PHOTORESISTOR_PIN, OUTPUT);
  pinMode(PHOTO_RES_1, INPUT);
  digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH);
}

int readPhotoresistor() {
  return analogRead(PHOTO_RES_1);
}

void setPhotoresistorThreshold(int threshold) {
  lightThreshold = threshold; 
}

bool checkLightThreshold(int lightValue) {
  if(readPhotoresistor() > lightThreshold){
    return true;
  }
  // Implement your light threshold logic here
  return false; // Placeholder
}