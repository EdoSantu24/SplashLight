//I2C
#include <Wire.h>
#define SDA_PIN 21
#define SCL_PIN 22
//https://docs.arduino.cc/language-reference/en/functions/communication/wire/

//The accelerometer
#include "GY521.h"
GY521 accelo(0x68);
//https://github.com/RobTillaart/GY521

//LCD (testbench)
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);

//photoresistor
#define PHOTO_RES_1 34

//https://projecthub.arduino.cc/tropicalbean/how-to-use-a-photoresistor-1143fd


//TP4056 (charger module) 
  //it has no library. however you may use its outputs to tell the arduino if the device is charging or not
  //you can use mosfets to control the charging - so that it stops charging when battery is fully charged.

char in_message[100]; //used to pass message to print between functions


// the following pins turn on and off power for the different sensors.
#define TURN_ON_ALL_SENSORS_PIN 13
#define TURN_ON_LCD_PIN 13
#define TURN_ON_ACCELEROMETER_PIN 27
#define TURN_ON_PHOTORESISTOR_PIN 26
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
int wire_begin = 0;
void setup_sensors(int bitmask){
  //Wire.begin is done here - and only once, because if its set to begin - even after Wire.end(), then the whole ESP32 softlocks.
  if(wire_begin == 0){
    Wire.begin();
    wire_begin = 1;    
  }
  //I2C enable ... has been discontinued. to be done here
  if(bitmask & 1 && (bitmask & 16) == 0){
    Serial.println("UUUUUUUUUUUUUUUUUHHHHHHHHHHHHHHHH");
     //Wire.begin(); // Seems to softlock the entire system when called more than once: 
  } // the turn off wire will occur at end of this function
  
  //LCD enable
  if(bitmask & 2 && (bitmask & 32) == 0){
    Serial.println("UUUUUUUUUUUUUUUUUHHHHHHHHHHHHHHHH");
    digitalWrite(TURN_ON_LCD_PIN,HIGH);
    lcd.begin();
    lcd.clear();
    lcd.backlight();      // Make sure backlight is on
    lcd.setCursor(0,0);
  }
  //LCD disable
  if((bitmask & 2) == 0 && bitmask & 32) 
  {
    Serial.println("AAAAAAAAAAAAAAAAAAHHHHHHHHHH");
    //lcd.end(); //not a function
    digitalWrite(TURN_ON_LCD_PIN,LOW);
  }

  //Accelerometer enable
  if(bitmask & 4 && (bitmask & 64) == 0){
        Serial.println("UUUUUUUUUUUUUUUUUHHHHHHHHHHHHHHHH");
    digitalWrite(TURN_ON_ACCELEROMETER_PIN,HIGH);
    accelo.begin();
    if(accelo.isConnected()){
      Serial.println("gy521 online");
    }
    Serial.println("HOWDY1");
    accelo.calibrate(100,0,0,false);
    accelo.setAccelSensitivity(0);
    accelo.setGyroSensitivity(0);
    accelo.setNormalize(false);
    accelo.setThrottle();
  }
  //Accelerometer is disabled
  if((bitmask & 4) == 0 && bitmask & 64){
    //accelo.end(); //not a function
    Serial.println("AAAAAAAAAAAAAAAAAAHHHHHHHHHH");
    accelo.reset();
    digitalWrite(TURN_ON_ACCELEROMETER_PIN,LOW);
  }

  //photoresistor enanble/disable
  if(bitmask & 8){

    digitalWrite(TURN_ON_PHOTORESISTOR_PIN,HIGH);
  }
  else {
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN,LOW);
  }

  //disable I2C - deprecated because Wire softlocks if Wire.begin() is called more than once.
  if((bitmask & 1) == 0 && bitmask & 16){
    Serial.println("AAAAAAAAAAAAAAAAAAHHHHHHHHHH");
    //Wire.end();
  }

  //this delay might be unnecessary
  Serial.println("DONE with setting up sensors!");
  Serial.println((bitmask & 4));
  Serial.println((bitmask & 4) == 0);
  Serial.println( bitmask & 64);
  Serial.println((bitmask & 4) == 0 && bitmask & 64);
  delay(1000);
}

/**This function holds the different calls for the different operational modes to enable the sensors.
  bit 0 (0x01)1 = I2C (needed for LCD and  accelerometer)
  bit 1 (0x02)2 = LCD
  bit 2 (0x04)4 = Accelerometer
  bit 3 (0x08)8 = photoresistor
  bit 4 (0x10)16 = will assume I2C has already been enabled
  bit 5 (0x20)32 = will assume that LCD has already been enabled 
  bit 6 (0x40)64 = will assume that accelerometer has already been calibrated  
  
NOTES:
  :The LED is not factored in
  :The battery status is not factored in
  :the Geolocation is not factored in
*/
void operational_sensor_setup_calls(int operational_mode){
  //device is turned on: (NOTE: does not handle the case where the accelerometer has been enabled during sleep mode)
  setup_sensors(4 + 1);
  //--- START OF ACTIVE MODE
  //Active mode - assuming that there is no sleep between device is turned on and active mode
  setup_sensors(64 + 16 + 8 + 4 + 1);
  //Active mode - assuming that it comes from parking mode.
  setup_sensors(64 + 16 + 8 + 4 + 1);
  //--- END   OF ACTIVE MODE
  
  //--- START OF PARKING MODE
  //parking mode - assuming that there is no sleep between device is turned on and parking mode
  setup_sensors(64 + 16 + 4 + 1);
    //parking mode - assuming that it comes from storage mode.
  setup_sensors(4 + 1);
  //--- END   OF PARKING MODE

  //--- START OF STORAGE MODE
  //Storage mode 
  setup_sensors(64 + 16);
  //--- END   OF STORAGE MODE


  
  

}



void setup() {
  
  Serial.begin(115200);
  //enable pins
  pinMode(TURN_ON_LCD_PIN,OUTPUT);
  pinMode(TURN_ON_ACCELEROMETER_PIN,OUTPUT);
  pinMode(TURN_ON_PHOTORESISTOR_PIN,OUTPUT);
  pinMode(PHOTO_RES_1,INPUT);
  //with pins enabled, we may start the sensors (NOTE THAT: out of deep sleep, i know not how exactly we should set the startup of the accelerometer: we might not have to recalibrate it, if it was turned on during deep sleep)
  setup_sensors(8 + 4 + 2 + 1);
  //the below is old setup function
  if(0){
    Wire.begin();
    //TESTBENCH:
    /*###########
      SETUP LCD #
    *///###########
    lcd.begin();
    lcd.clear();
    lcd.backlight();      // Make sure backlight is on
  
    //SENSORS:
    /*#####################
      SETUP Accelerometer #
    *///#####################                
    accelo.begin();
    if(accelo.isConnected()){
      Serial.println("gy521 online");
    }
    delay(1000);
    if(accelo.isConnected()){
      Serial.println("gy521 online");
    }
    Serial.println("HOWDY1");
    accelo.calibrate(100,0,0,false);
    accelo.setAccelSensitivity(0);
    accelo.setGyroSensitivity(0);
    accelo.setNormalize(false);
    accelo.setThrottle();
    /*##############################
    SETUP pins for Photoresistor #
    *///##############################
    pinMode(PHOTO_RES_1,INPUT);
    //NON-SENSORS:

    /*#########################################
      SETUP 03962a Ion battery charger module #
    *///#########################################
    //TBD
  }
  
  

  

}

/*##############
  FUNCTIONS LCD #
*///##############

  //convert to compatible text
  
  //write text.
void write_text(byte* payload, unsigned int length) {  
  Serial.println("<Message arrived>"); 
  Serial.print("\t");
  lcd.clear();
  lcd.setCursor(1,0);   //Column, Row
  int i = 0; 
  for (i = 0; i < length; i++) { 
    Serial.print((char)payload[i]); 
    lcd.print((char)payload[i]);
  } 
  Serial.println("");
  Serial.println("<\Message arrived>");
}
  //convert types
int int_to_text(int val){
  String str = String(val);
  str.toCharArray(in_message, 100);
  return str.length();
}

  //append text
int collumn = 0;
int row = 0;
void append_lcd(char* payload, unsigned int length){
  for (int i = 0; i < length; i++) { 
    Serial.print((char)payload[i]); 
    lcd.print((char)payload[i]);
    collumn++;
    if(collumn >= 16){
      collumn = 0;
      row++;
      lcd.setCursor(collumn,row);
    }
  } 
  
}
  //clear text
void clear_lcd(){
  lcd.clear();
  lcd.setCursor(0,0);
  collumn = 0;
  row = 0;
}
  //turn on plain, FIFO, FIFO with new line, panel back-and-forth, and panel back-and forth with new line, 

  //parse accelometer

//WILL PRINT AS INT RATHER THAN FLOAT.
void float_to_string_simpl(float input){
  String tim = String((int)input);
  tim += "|";
  int len = tim.length();
  tim.toCharArray(in_message,100); 
  append_lcd(in_message,len);
  
}

void print_accelometer_res(){
  Serial.println("<print accelometer>"); 
  float ax = accelo.getAngleX();
  float ay = accelo.getAngleY();
  float az = accelo.getAngleZ();
  float temp = accelo.getTemperature();
  float gx = accelo.getGyroX();
  float gy = accelo.getGyroY();
  float gz = accelo.getGyroZ();
  float_to_string_simpl(ax);
  float_to_string_simpl(ay);
  float_to_string_simpl(az);
  float_to_string_simpl(temp);
  float_to_string_simpl(gx);
  float_to_string_simpl(gy);
  float_to_string_simpl(gz);
  Serial.println("");

}

//SENSORS:

/*########################
  FUNCTION Accelerometer #
*///########################

  //function for read data

  //modify threshold

  //react to sufficient threshold of accelerometer

  //react to stop of accelerometer change - enable timer or count, so that it will tally whenever the bike is likely to be idle

/*#################################
  FUNCTION pins for Photoresistor #
*///#################################

  //read data

  //modify threshold.

  //react to a sufficient threshold of light

  //react to a sufficient threshold of darkness

//NON-SENSORS:

/*############################################
  FUNCTION 03962a Ion battery charger module #
*///############################################

  //read charge

  //modify charging strength

  //turn on charging

  //react to connection of battery

  //react to disconnection of battery

  //react to battery being overcharged

  //react to unidentifiable battery (?)



//so far: this is where we test our program
int iter = 0; // this variable is used to test turning on and off sensors
void loop() {
  iter++;
  Serial.print("iter is:");
  Serial.println(iter);
  //the first cycle, the sensors are turned on, the next  
  if(iter >= 2){
    
    if(iter > 5){
      setup_sensors(64 + 32 + 16 + 8 + 4 + 2 + 1);
    }
    else if (iter == 5){
      setup_sensors(8 + 4 + 2 + 1);
    }
    else if (iter == 2) {
      setup_sensors(64 + 32 + 16);
    }
    else{
      setup_sensors(0);
    }
  }

  Serial.println("HOWDY2");
  String hello = "hello there";
  hello.toCharArray(in_message, 100);
  write_text((byte*)in_message,hello.length());
  delay(1000);
  int len = int_to_text(12456);
  write_text((byte*)in_message,len);
  delay(1000);
  accelo.read();
  float ax = accelo.getAngleX();
  float ay = accelo.getAngleY();
  float az = accelo.getAngleZ();
  float temp = accelo.getTemperature();
  float gx = accelo.getGyroX();
  float gy = accelo.getGyroY();
  float gz = accelo.getGyroZ();
  Serial.print(ax); Serial.print("|");Serial.print(ay); Serial.print("|");Serial.print(az); Serial.print("|");
  Serial.println("");
  Serial.print(temp); Serial.print("|");
  Serial.println("");
  Serial.print(gx); Serial.print("|");Serial.print(gy); Serial.print("|");Serial.print(gz); Serial.print("|");
  Serial.println("");
  clear_lcd();
  print_accelometer_res();
  delay(3000);
  int light = analogRead(PHOTO_RES_1);
  Serial.print("light strength: ");
  Serial.println(light);
  len = int_to_text(light);
  write_text((byte*)in_message,len);
  delay(1000);
  Serial.println("-----------------------");


}


/*!TO DO LIST:
HOW TO HANDLE IT BEING AT A STOP LIGHT
HOW TO HANDLE TURN ON WHEN MOVED IN STORAGE MODE
HOW TO HANDLE TURN ON WHEN MOVED IN PARKED MODE





*/
