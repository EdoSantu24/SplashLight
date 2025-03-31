

#include "GY521.h"
#include <Wire.h>
//https://github.com/RobTillaart/GY521
//https://docs.arduino.cc/language-reference/en/functions/communication/wire/

#define SDA_PIN 21
#define SCL_PIN 22

GY521 accelo(0x68);

//photoresistor
//https://projecthub.arduino.cc/tropicalbean/how-to-use-a-photoresistor-1143fd
#define PHOTO_RES_1 34
#define PHOTO_RES_2 12


//TP4056 (charger module) 
  //it has no library. however you may use its outputs to tell the arduino if the device is charging or not
  //you can use mosfets to control the charging - so that it stops charging when battery is fully charged.

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);
char msg[75];
char rec[75];
unsigned long lastMsg = 0;
char in_message[100]; //used to move incoming message into main loop if needed


void setup() {
  //"TESTBENCH":
  Serial.begin(115200);
  Serial.println("HOWDY0");
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
  Wire.begin();                
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
  pinMode(PHOTO_RES_2,OUTPUT);
  digitalWrite(PHOTO_RES_2,HIGH);
  //NON-SENSORS:

  /*#########################################
    SETUP 03962a Ion battery charger module #
  *///#########################################

}

//"TESTBENCH":

/*##############
  FUNCTION LCD #
*///##############

  //convert to compatible text
  
  //write text.
void write_text(byte* payload, unsigned int length) {

  
    // debugging message at serial monitor 
  Serial.println("<Message arrived>"); 
  Serial.print("\t");
  lcd.clear();
  lcd.setCursor(1,0);   //Column, Row
  int i = 0; 
  for (i = 0; i < length; i++) { 
    rec[i] = (char)payload[i];
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

void loop() {
  //so far: this is where we test our program
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
