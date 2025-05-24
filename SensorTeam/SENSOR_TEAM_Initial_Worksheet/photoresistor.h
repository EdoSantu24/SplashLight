

#ifndef PHOTORESISTOR_H
#define PHOTORESISTOR_H

#define PHOTO_RES_1 0
//#define TURN_ON_PHOTORESISTOR_PIN 26

void setupPhotoresistor();
int readPhotoresistor();
void setPhotoresistorThreshold(int threshold);
bool checkLightThreshold(int lightValue);

#endif
