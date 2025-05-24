/******************************************
 *Website: www.elegoo.com
 * 
 *Time:2018.1.23
 *
 ******************************************/

#include "pitches.h"
#define SOUND_PIN 25
 
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

// // Define pin for buzzer
// const int buzzerPin = 25; // you can change this to any free GPIO pin

// void setup() {
//   // Setup PWM for the buzzer
//   ledcSetup(0, 2000, 8);  // Channel 0, 2000 Hz frequency, 8-bit resolution
//   ledcAttachPin(buzzerPin, 0); // Attach buzzerPin to channel 0
// }

// void loop() {
//   // Play tone
//   ledcWrite(0, 128); // 50% duty cycle (range 0-255)

//   delay(500); // sound for 500 ms

//   // Stop tone
//   ledcWrite(0, 0); // 0% duty cycle (silent)

//   delay(500); // silent for 500 ms
// }

