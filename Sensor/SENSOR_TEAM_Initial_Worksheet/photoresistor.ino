/*******************************************************
 * File:        photoresistor.ino
 * Description: this file contains the setup, read, threshold  set, and check functions for the photoresistor
 * Team:        Sensor Team
 * Created:     2025-04-07
 * Modified:    2025-05-04 (comments)
 *
 * License:     the structure of THIS PARTICULAR COMMENT was made using Chatgpt 
                 (we deemed it to be quite good looking)
                all else in this file has been made by the Sensor Team.
 *******************************************************/

#include "photoresistor.h"
int lightThreshold = 500; //#1


/*###
This function sets up the photoresistor
  One thing to note: this needs to be updated to NOT use a "TURN ON" pin. it should always be on - as it uses very little power... and we do not have the spare pins

arguments: none

returns: void
#######*/
void setupPhotoresistor() {
  pinMode(TURN_ON_PHOTORESISTOR_PIN, OUTPUT);
  pinMode(PHOTO_RES_1, INPUT);
  digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH);
}
/*###
This function reads the output making it through the photoresistor

arguments: none

returns: an int which corresponds to the analog, turned digital, value: the higher, the more bright it was.
#######*/
int readPhotoresistor() {
  return analogRead(PHOTO_RES_1);
}

/*###
This function reads the output making it through the photoresistor.
 #1 - might need an upper and lower threshold (so it do not switch constantly between LIGHT and DARK)

arguments: 
  int threshold: the new threshold to set

returns: void
#######*/
void setPhotoresistorThreshold(int threshold) {
  lightThreshold = threshold; //#1 //also fixed the misspelling from "threshhold" to "threshold"
}

/*###
This function checks if the light measured now exceeds the threshold
 #1 - might need an upper and lower threshold (so it do not switch constantly between LIGHT and DARK)

arguments: 
  int lightValue: residual argument

returns: a bool, which tells if it is BIGHT or DARK.
#######*/
bool checkLightThreshold(int lightValue) {
  if(readPhotoresistor() > lightThreshold){ //#1
    return true;
  }
  // Implement your light threshold logic here
  return false; // Placeholder
}
