/*This implementation uses the new pin-setup. here is my attempt at explaining:
//NOTE: transistors are PN 2222A 901 - At least thats the ones i used (unsure if they had 901 at the end)
//NOTE: (FOR !!!OSKAR!!!): It seems that the transistor does not make this system work. 
  //we instead need to use MOSFET's instead of transistors. 
  //(THEREFORE PLEASE JUST CONNECT PHOTORESISTORS AND THE VCC OF ACCELEROMETER AND THE VCC OF LCD SCREEN TO THE VCC_TRACK_1 INSTEAD!

//MESSAGE FOR OSKAR ABOVE (TL:DR: DO NOT USE TRANSISTORS. JUST CONNECT VCC TO VCC ON SENSORS)
||<VCC>
VIN --> VCC_TRACK_1

||<GND>
GND --> GND_TRACK_1

||<SDA+SCL (ESP32)>
#define SDA_PIN 21
#define SCL_PIN 22
D21 --> VCC_TRACK_2
D22 --> GND_TRACK_2

||<PHOTORESISTOR>
#define PHOTO_RES_1 34
#define TURN_ON_PHOTORESISTOR_PIN 26
VCC_TRACK_1 --> TRANSISTOR_PIN_1(FLAT_SIDE,LEFT) -- TRANSISTOR_PIN_3(FLAT_SIDE,RIGHT) --> PhotoRES(either_end)-->D34
                                                                                      L-->RES->GND_TRACK_1 //ENABLES PULLDOWN
D26 --> TRANSISTOR_PIN_2(MIDDLE)

||<Accelerometer>
#define TURN_ON_ACCELEROMETER_PIN 27
GND_TRACK_1 --> GND(ACCELEROMETER)
VCC_TRACK_1 --> TRANSISTOR_PIN_1(FLAT_SIDE,LEFT) -- TRANSISTOR_PIN_3(FLAT_SIDE,RIGHT) --> VCC(ACCELEROMETER)
D27 --> TRANSISTOR_PIN_2(MIDDLE)
SDA(ACCELEROMETER) --> VCC_TRACK_2
SCL(ACCELEROMETER) --> GND_TRACK_2

||<LCD>
//NOTE: This one does not work in the current setup - more details are given later in this file.
#define TURN_ON_LCD_PIN 13 
GND_TRACK_1 --> GND(LCD)
VCC_TRACK_1 --> TRANSISTOR_PIN_1(FLAT_SIDE,LEFT) -- TRANSISTOR_PIN_3(FLAT_SIDE,RIGHT) --> VCC(LCD)
D13 --> TRANSISTOR_PIN_2(MIDDLE)
SDA(LCD) --> VCC_TRACK_2
SCL(LCD) --> GND_TRACK_2
                    


*/

//I2C
#include <Wire.h>
//Not really used - but those are the Dx pins which correspond to SDA and SCL
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

//Accelerometer values:
float accelThreshold = 0.2;  //in m/s²
unsigned long idleTimeout = 30000; //30 sec timeout 
unsigned long lastMovementTime = 0;
bool isMoving = false;


struct AccelData {
  float ax, ay, az;  // Acceleration (m/s²)
  float gx, gy, gz;  // Gyro (deg/s)
  float angleX, angleY, angleZ; // Angles
};
// the following pins turn on and off power for the different sensors.
  /*this pin seems not to work - or rather, i cannot make the LCD an entity that can be enabled and disabled
  'also, if you power it directly using VCC, then the accelerometer will ALWAYS be on
  */
  #define TURN_ON_LCD_PIN 13 
  //turns on the accelerometer. I used  PN 2222A 901 Transistors, - flat side, left to right, 1 = emitter, 2 = base, 3 = collector 
  //https://www.alldatasheet.com/datasheet-pdf/view/18722/PHILIPS/PN2222A.html
  #define TURN_ON_ACCELEROMETER_PIN 27
  #define TURN_ON_PHOTORESISTOR_PIN 26

int wire_begin = 0; // relevant for setup_sensors()
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
  //---//
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
  //---//
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
  //---//
  //photoresistor enanble/disable
  if(bitmask & 8){

    digitalWrite(TURN_ON_PHOTORESISTOR_PIN,HIGH);
  }
  else {
    digitalWrite(TURN_ON_PHOTORESISTOR_PIN,LOW);
  }
  //---//
  //disable I2C - deprecated because Wire softlocks if Wire.begin() is called more than once.
  if((bitmask & 1) == 0 && bitmask & 16){
    Serial.println("AAAAAAAAAAAAAAAAAAHHHHHHHHHH");
    //Wire.end();
  }

  //this delay is unnecessary
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

//--SETUP--//

void setup() {
  
  Serial.begin(115200);
  //enable pins
  pinMode(TURN_ON_LCD_PIN,OUTPUT);
  pinMode(TURN_ON_ACCELEROMETER_PIN,OUTPUT);
  pinMode(TURN_ON_PHOTORESISTOR_PIN,OUTPUT);
  pinMode(PHOTO_RES_1,INPUT);
  //with pins enabled, we may start the sensors (NOTE THAT: out of deep sleep, i know not how exactly we should set the startup of the accelerometer: we might not have to recalibrate it, if it was turned on during deep sleep)
  setup_sensors(8 + 4 + 2 + 1);
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

/**
this function gets the accelerometers data and prints it
*/
void print_accelometer_res(){
  Serial.println("<print accelometer>"); 
  //the orientation compared to gravity
  float ax = accelo.getAngleX();
  float ay = accelo.getAngleY();
  float az = accelo.getAngleZ();
  //have no idea why this is a feature
  float temp = accelo.getTemperature();
  //the acceleration - change in movement.
  float gx = accelo.getGyroX();
  float gy = accelo.getGyroY();
  float gz = accelo.getGyroZ();
  //print all data - because floats a tad annoying (especially when printing on LCD)
  float_to_string_simpl(ax);
  float_to_string_simpl(ay);
  float_to_string_simpl(az);
  float_to_string_simpl(temp);
  float_to_string_simpl(gx);
  float_to_string_simpl(gy);
  float_to_string_simpl(gz);
  //new line, because the above calls make no new line.
  Serial.println("");

}

//SENSORS:

/*########################
  FUNCTION Accelerometer #
*///########################
//FOR THIS ONE: CHECK THE print_accelometer_res() FUNCTION!
  //function for read data

AccelData readAccelerometerData() {
  //jeg laver en struct der holder alt informationen. Bruger kun ax ay tho.
  AccelData data;
  accelo.read();
  
  data.ax = accelo.getAccelX();
  data.ay = accelo.getAccelY();
  data.az = accelo.getAccelZ();
  
  data.gx = accelo.getGyroX();
  data.gy = accelo.getGyroY();
  data.gz = accelo.getGyroZ();
  
  data.angleX = accelo.getAngleX();
  data.angleY = accelo.getAngleY();
  data.angleZ = accelo.getAngleZ();
  
  return data;
}
  //modify threshold
void setAccelerometerThresholds(float newThreshhold, float newTimeout){
  //opdatere threshholds
  accelThreshold = newThreshhold;
  idleTimeout = newTimeout;
  Serial.print("New thresholds set - Accel: ");
  Serial.print(accelThreshold);
  Serial.print("Idle timeout: ");
  Serial.print(idleTimeout/1000);
  Serial.println("s");
  }
  //react to sufficient threshold of accelerometer
bool checkMovementThreshold(AccelData data){
  unsigned long previousTime = 0;
  //funktionen returnere true når farten er mindre end threshhold altså at den holder stille.
  //jeg regner ecludian distance ud for at få hastigheden i et 2d space. Bruger ikke z da jeg tænker ikke den er sp
  float speedDiff  = sqrt(sq(data.ax) + sq(data.ay));
  if(speedDiff > accelThreshold){
    return true;
  }
  else {
    return false;
  }
}
void reactToMovement() {
  //hvis der er bevægelse fra stille stand
  if (!isMoving) {
    isMoving = true;
    lastMovementTime = millis();
    
    // Actions when movement starts
    Serial.println("Movement detected!");
  } else {
    // Update last movement time
    lastMovementTime = millis();
  }
}
void reactToIdle() {
  //hvis der er stille stand i længere end threshhold
  if (isMoving) {
    isMoving = false;
    
    unsigned long idleDuration = (millis() - lastMovementTime) / 1000;
    Serial.print("Bike idle for ");
    Serial.print(idleDuration);
    Serial.println(" seconds");
  }
}


void monitorAccelerometer() {
  AccelData currentData = readAccelerometerData();
  //holder øje med hvor lang tid vi har været stille
  if (checkMovementThreshold(currentData)) {
    reactToMovement();
  } 
  //hvis den er stille stående og den har været det i idleTImeout (30 sec default ) så reagere på idle
  else if (millis() - lastMovementTime > idleTimeout) {
    reactToIdle();
  }
}

/*#################################
  FUNCTION pins for Photoresistor #
*///#################################
//FOR THIS ONE: you litterally just perform analog read on the appropiate pin
  //read data

  //modify threshold.

  //react to a sufficient threshold of light

  //react to a sufficient threshold of darkness

//NON-SENSORS:

/*############################################
  FUNCTION 03962a Ion battery charger module #
*///############################################
//NOT OUR PROBLEM (Sensor team).
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
  //the first cycle, the sensors are turned on.
  if(iter >= 2){
    //keep the sensor on - tests if "is already on" works
    if(iter > 5){
      setup_sensors(64 + 32 + 16 + 8 + 4 + 2 + 1);
    }// the below: this turns sensors back on
    else if (iter == 5){
      setup_sensors(8 + 4 + 2 + 1);
    }// the below: this turns sensors off
    else if (iter == 2) {
      setup_sensors(64 + 32 + 16);
    }//keep the sensor off - ensure that it wont try to turn off sensors that are already turned off
    else{
      setup_sensors(0);
    }
  }
  //print string on LCD.
  String hello = "hello there";
  hello.toCharArray(in_message, 100);
  write_text((byte*)in_message,hello.length());
  delay(1000);
  //turn int to string for LCD.
  int len = int_to_text(12456);
  write_text((byte*)in_message,len);
  delay(1000);
  //READ accelo's data
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
  //Now try to read accelometer, and write it on the LCD.
  clear_lcd();
  print_accelometer_res();
  delay(3000);
  //Read photoresistor and print on LCD
  int light = analogRead(PHOTO_RES_1);
  Serial.print("light strength: ");
  Serial.println(light);
  len = int_to_text(light);
  write_text((byte*)in_message,len);
  delay(1000);
  //Split this iterations output from future iterations.
  Serial.println("-----------------------");
//accelerometer koden
  monitorAccelerometer();

}


/*!TO DO LIST:
HOW TO HANDLE IT BEING AT A STOP LIGHT
HOW TO HANDLE TURN ON WHEN MOVED IN STORAGE MODE
HOW TO HANDLE TURN ON WHEN MOVED IN PARKED MODE





*/