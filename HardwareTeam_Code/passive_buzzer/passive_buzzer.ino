/******************************************
 *Website: www.elegoo.com
 * 
 *Time:2018.1.23
 *
 ******************************************/

#include "pitches.h"
#define SOUND_PIN 23
 
// notes in the melody:
int melody[] = 
  {
  NOTE_E6, NOTE_DS6, NOTE_E6, NOTE_DS6, NOTE_E6, NOTE_B5, NOTE_D6, NOTE_C6, NOTE_A5
  };
int duration = 500;  // 500 miliseconds
 
void setup() {
 
}
 
void loop() {  
  for (int thisNote = 0; thisNote < 9; thisNote++) {
    // pin8 output the voice, every scale is 0.5 sencond
    tone(SOUND_PIN, melody[thisNote], duration);
     
    // Output the voice after several minutes
    delay(100);
  }
   
  // restart after two seconds 
  delay(2000);
}
