// Libraries
#include <WiFi.h>
#include <Wire.h>
#include <pitches.h>
// #include <LoRa.h> // LoRa libraries
// #include <Adafruit_Sensor.h> // Example for accelerometer
// #include <Adafruit_LIS3DH.h> // Example accelerometer

//for the led, there is a variable called ledState that keeps track of the status of the led, please put it true/false, when you update the status of the led

//START LoraWAN Setups

#include "LoRaWan_APP.h"

/* OTAA keys */
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC8 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x99 };
uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x45 };

/* ABP (solo se usi ABP invece di OTAA) */
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
uint32_t appTxDutyCycle = 15000; // ogni 30s
bool loraWanAdr = true; 
bool isTxConfirmed = true; 
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

bool ledState = false;

bool gpsRequestPending = false;
int counter = 1;

unsigned long lastLedToggle = 0;
unsigned long lastTx = 0;
const unsigned long toggleInterval = 60000;   // 60s
const unsigned long txInterval = 15000;       // 15s => sending every 15 seconds

//END of LoraWAN setups

// Define pins
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER
#define VOLTAGE_PIN 4
#define LED_PIN 5
#define SOUND_PIN 18
#define ACCELO_PIN 19
#define LIGHT_SENS_PIN 34
#define TURN_ON_ACCELEROMETER_PIN 6
#define TURN_ON_PHOTORESISTOR_PIN 7
// #define WIFI_PIN 21 // optional if needed to enable WiFi manually

// Constants for battery status calculation
const float R1 = 14700.0;          // 14.7kΩ
const float R2 = 20000.0;          // 20kΩ
const float ADC_CORRECTION = 0.8;  // Calibration factor

// Variables
bool active = false;
bool parked = true;

// packet to send every 15s
static void prepareTxFrame(uint8_t port) {
  if (counter <=3) {
    appDataSize = 1;
    appData[0] = ledState ? 0x0B : 0x0C;
    counter = counter+1;}
  else {
    appDataSize = 1;
    appData[0] = readBattery();
    counter = 0;
  }
}
//Downlinkhandle for sending LoraWAN
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  if (mcpsIndication->RxData == true && mcpsIndication->BufferSize > 0) {
    Serial.print("Downlink ricevuto: ");
    for (int i = 0; i < mcpsIndication->BufferSize; i++) {
      Serial.print("0x");
      Serial.print(mcpsIndication->Buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    uint8_t command = mcpsIndication->Buffer[0];

    switch (command) {
      case 0x02:
        Serial.println("Command recieved: activeMode");
        activeMode();
        break;

      case 0x03:
        Serial.println("Command recieved: parkingMode");
        parkingMode();
        break;

      case 0x04:
        Serial.println("Command recieved: storageMode");
        storageMode();
        break;

      case 0x05:
        Serial.println("Command recieved: request_gps");
        request_gps();
        break;

      default:
        Serial.print("Commando unknown: 0x");
        Serial.println(command, HEX);
        break;
    }
  }
}

void setup() {
  // Set up Serial
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  while (!Serial) {
    delay(10);
  }
  analogReadResolution(12); // 12-bit ADC
  analogSetAttenuation(ADC_11db); // 0-3.3V range

  // Set up pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOUND_PIN, OUTPUT);
  pinMode(ACCELO_PIN, INPUT);
  pinMode(LIGHT_SENS_PIN, INPUT);

  // Setup WiFi
  setupWiFi();

  // Setup LoRa
  setupLoRa();

  // Setup Accelerometer (if necessary)
  setupAccelerometer();

  // Setup Button if needed
  // pinMode(BUTTON_PIN, INPUT_PULLUP); // Example if using a physical button

  parkingMode(); // Start in parking mode
}
//LoraWAN is sent in loop
void loop() {
  unsigned long currentMillis = millis();

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;

    case DEVICE_STATE_SEND:
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;

    case DEVICE_STATE_CYCLE:
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
      // Gestione manuale del timing
      if (currentMillis - lastLedToggle >= toggleInterval) {
        lastLedToggle = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        Serial.print("LED toggled: ");
        Serial.println(ledState ? "ON" : "OFF");
      }

      if (currentMillis - lastTx >= txInterval) {
        lastTx = currentMillis;
        deviceState = DEVICE_STATE_SEND;
      }

      LoRaWAN.sleep(loraWanClass);
      break;

    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}

void activeMode() {
  Serial.println("Entering Active Mode");

  digitalWrite(LED_PIN, LOW); // initially turning off LED
  ledState = false;
  digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW); //turning on accelerometer
  digitalWrite(TURN_ON_PHOTORESISTOR_PIN, LOW); //turning on photoresistor
  setupWiFi(); // setting up WiFi
  setupLoRa(); // setting up LoRa

  active = true;
  parked = false;

  while (active) {
    float battery = readBattery();

    if (battery <= 20.0) {
      // Critical battery warning
      static unsigned long lastWarningAct = 0;
      if (millis() - lastWarningAct > 30 * 1000) { // every 30 sec
        playCriticalBatteryWarning();
        sendMsg("BatteryWarning");
        // sendBatteryWarning();
        lastWarningAct = millis();
      }
    }

    bool lightDetected = checkLight() ;
    if (lightDetected) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    if (!movementDetected()) {
      Serial.println("No movement detected. Starting countdown.");
      unsigned long countdownStart = millis();

      while (millis() - countdownStart < 30000) { // 30s countdown
        if (movementDetected()) {
          Serial.println("Movement detected, resetting active mode.");
          break;
        }
        checkIncomingMessage();
        delay(3000); // Check every 3 seconds
      }
      if (!movementDetected()) {
        Serial.println("No movement after countdown. Switching to Parking Mode.");
        parkingMode();
        break;
      }
    }
    delay(1000);
  }
}

void parkingMode() {
  Serial.println("Entering Parking Mode");

  digitalWrite(LED_PIN, LOW); // initially turning off LED
  ledState = false;
  digitalWrite(TURN_ON_ACCELEROMETER_PIN, LOW); //turning on accelerometer
  digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH); //turning off photoresistor
  setupWiFi(); //setting up wifi
  setupLoRa(); //setting up LoRa

  active = false;
  parked = true;

  while (parked) {
    float battery = readBattery();
    if (battery <= 20.0) {
      static unsigned long lastWarningPark = 0;
      if (millis() - lastWarningPark > 15 * 60 * 1000) { // every 15 min
        sendMsg("BatteryWarning");
        // sendBatteryWarning(); // Send warning through LoRa
        lastWarningPark = millis();
      }
    }

    if (movementDetected()) {
      Serial.println("Movement detected. Switching to Active Mode.");
      activeMode();
      break;
    }
    checkIncomingMessage();
    delay(1000);
  }
}

void storageMode() {
  Serial.println("Entering Storage Mode");

  digitalWrite(LED_PIN, LOW); //turning off LED
  ledState = false;
  digitalWrite(TURN_ON_ACCELEROMETER_PIN, HIGH); //turning off accelerometer
  digitalWrite(TURN_ON_PHOTORESISTOR_PIN, HIGH); //turning off photoresistor
  setupWiFi(); //setting up wifi
  setupLoRa(); //setting up LoRa
  
  active = false;
  parked = false;

  while (!active && !parked) {
    float battery = readBattery();
    if (battery <= 20.0) {
      static unsigned long lastWarningStore = 0;
      if (millis() - lastWarningStore > 60 * 60 * 1000) { // every hour
        sendMsg("BatteryWarning");
        // sendBatteryWarning(); // Send warning through LoRa
        lastWarningStore = millis();
      }
    }
    checkIncomingMessage();
    delay(1000);
  }
}

////// HELPER FUNCTIONS //////

float readBattery() {
  int raw = analogRead(VOLTAGE_PIN);
  float vPin = (raw * ADC_CORRECTION * 3.3 / 4095.0);
  float batteryVoltage = vPin * (R1 + R2) / R2;
  float batteryCharge = mapBatteryPercentage(batteryVoltage);

  Serial.print("Charge: ");
  Serial.print(batteryCharge);
  Serial.print("% | Raw ADC: ");
  Serial.print(raw);
  Serial.print(" | Voltage: ");
  Serial.println(vPin);

  return batteryCharge;
}

float mapBatteryPercentage(float v) {
  if (v >= 4.20) return 100.0;
  else if (v >= 4.10) return 90.0;
  else if (v >= 3.95) return 80.0;
  else if (v >= 3.80) return 70.0;
  else if (v >= 3.70) return 60.0;
  else if (v >= 3.60) return 50.0;
  else if (v >= 3.50) return 40.0;
  else if (v >= 3.40) return 30.0;
  else if (v >= 3.20) return 20.0;
  else if (v >= 3.00) return 10.0;
  else if (v >= 2.75) return 1.0;
  else return 0.0;
}

void playCriticalBatteryWarning() {
  Serial.println("Critical battery! Playing audio warning.");
  int melody = NOTE_E6;
  int duration = 250;
  tone(SOUND_PIN, melody, duration);
  delay(250);
  tone(SOUND_PIN, melody, duration);
}

bool movementDetected() {
  // Simple dummy example: Read accelerometer pin
  int movement = digitalRead(ACCELO_PIN);
  return movement == HIGH; // adjust depending on your accelerometer logic
}

void checkIncomingMessage() {
  sendMsg("LEDStatus");
  // sendLEDStatus();

// TODO: Replace this dummy with actual message checking via WiFi, LoRa, etc.
  if (Serial.available() > 0) { // Example: read from Serial for testing
    String incoming = Serial.readStringUntil('\n');
    incoming.trim(); // Remove whitespace

    Serial.printf("Received message: %s\n", incoming.c_str());

    if (incoming == "appToActivemode") {
      activeMode();
    } else if (incoming == "appToParkmode") {
      parkingMode();
    } else if (incoming == "appToStoragemode") {
      storageMode();
    } else if (incoming == "appBatteryUpdate") {
      sendMsg("BatteryUpdate");
      // sendBatteryUpdate();
    } else if (incoming == "appLocationUpdate") {
      sendMsg("LocationUpdate");
      // sendLocationUpdate();
    }
  }
}

bool checkLight(){
 int lightLevel = analogRead(LIGHT_SENS_PIN);
  Serial.printf("Light sensor reading: %d\n", lightLevel);
  return lightLevel > 2000; // tune threshold
}

// Encode 2 floats (lat, lon) into 8 bytes (4 bytes each, IEEE 754)
void send_gps(float latitude, float longitude) {
  Serial.println("Sending GPS coordinates to TTN...");

  uint8_t *latPtr = (uint8_t*) &latitude;
  uint8_t *lonPtr = (uint8_t*) &longitude;

  appDataSize = 8;
  for (int i = 0; i < 4; i++) appData[i] = latPtr[i];
  for (int i = 0; i < 4; i++) appData[i + 4] = lonPtr[i];

  LoRaWAN.send();
}

//the code for the GPS has to be inserted here!!! for now just hardcoded
void request_gps() {
  Serial.println("GPS request triggered. Getting fake coordinates...");
 
  float fake_lat = 55.6761;  // Example: Copenhagen latitude
  float fake_lon =  12.5683;  // Example: Copenhagen longitude
 
  counter = 0;

  send_gps(fake_lat, fake_lon);
}


void setupAccelerometer() {
  Serial.println("Setting up accelerometer...");
  // TODO: Initialize accelerometer if needed
}

void sendMsg(String whatToSend) {
  if      (whatToSend == "BatteryUpdate"){

  } 
  else if (whatToSend == "BatteryWarning"){

  }
  else if (whatToSend == "LocationUpdate") {
  
  } 
  else if (whatToSend == "LEDStatus") {
  
  }

}
