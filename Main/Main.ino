//# LIST OF ISSUES
//#1 - Lacks functions definition - we call a function that does not exist
void setupWiFi(){
  return;
}
void setupLoRa(){
  return;
}
//#1 STATUS: UNSOLVED



// Libraries
#include <WiFi.h>
#include <Wire.h>
#include <../libraries/pitches/pitches.h>
#include <../Sensor/SENSOR_TEAM_Initial_Worksheet/accelerometer.h>
#include <../Sensor/SENSOR_TEAM_Initial_Worksheet/photoresistor.h>
//#include <loramac/LoRaMac.h>
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

/* ABP keys (not used)*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
/* LoRaWAN configuration */
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
uint32_t appTxDutyCycle = 15000; 
bool loraWanAdr = true; 
bool isTxConfirmed = true; 
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;
/* Startup sequence flags and counters */
bool firstJoinDone = false;
uint8_t startupMessageStep = 0;


bool ledState = false;
/* GPS request flag */
bool gpsRequestPending = false;
/* Transmission control */
int counter = 1;

unsigned long lastTx = 0;
const unsigned long txInterval = 15000;       // Data TX every 30s

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

/**
 * Prepares the application payload for regular transmissions.
 * Alternates between sending LED state and battery level.
 */
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
/**
 * Handles downlink messages received from the network server.
 * Recognizes and processes remote commands for mode switching or GPS request.
 */
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
/**
 * Initial device setup.
 */
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
  setupWiFi(); //#1

  // Setup LoRa
  setupLoRa(); //#1

  // Setup Accelerometer (if necessary)
  setupAccelerometer();

  // Setup Button if needed
  // pinMode(BUTTON_PIN, INPUT_PULLUP); // Example if using a physical button

  parkingMode(); // Start in parking mode
}
/**
 * Main device loop. Manages state transitions, message sending, LED toggling, and sleep cycles.
 */
void loop() {
  unsigned long currentMillis = millis();

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      firstJoinDone = true; // Appena completato il join
      startupMessageStep = 0;
      break;

    case DEVICE_STATE_SEND:
       // Initial startup sequence: send LED, mode, battery
      if (firstJoinDone && startupMessageStep < 6) {
        switch (startupMessageStep) {
          case 0: // send the led state
            appDataSize = 1;
             appData[0] = ledState ? 0x0B : 0x0C; //hex 0B: sending LED ON
            break;                               //hex 0C: sending LED OFF
          case 1: // send the led state
            appDataSize = 1;
            appData[0] = ledState ? 0x0B : 0x0C;
            break;
          case 2: // send the mod
            appDataSize = 1;
            appData[0] = req_mod();
            break;
          case 3: // send the mod
            appDataSize = 1;
            appData[0] = req_mod();
            break;
          case 4: // send battery state
            appDataSize = 1;
            appData[0] = readBattery();
            break;
          case 5: // send battery state
            appDataSize = 1;
            appData[0] = readBattery();
            break;
          }

        if (startupMessageStep != 5) {
          LoRaWAN.send();
        }

        startupMessageStep++;

        if (startupMessageStep >= 6) {
              firstJoinDone = false; 
        }

        deviceState = DEVICE_STATE_CYCLE;
      } else {
          prepareTxFrame(appPort);
          LoRaWAN.send();
          deviceState = DEVICE_STATE_CYCLE;
        }
        break;

    case DEVICE_STATE_CYCLE:
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
      

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

float req_mod() {
  if (active == true) {
    return 0x02; //hex 02 is active
  }
  else if (parked == true) {
    return 0x03; //hex 03 is parked
  } 
  else {
    return 0x04; //hex 04 is storage
  }
}

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

/**
 * @brief Sends GPS coordinates encoded in IEEE 754 format to TTN.
 * @param latitude Latitude value (float)
 * @param longitude Longitude value (float)

 */
void request_gps() {
  Serial.println("GPS request triggered. Getting fake coordinates...");
 
    

  // this hardcoded latitude and longitude need to be replaced with the actual latitude and longitude retrieved by the GNSS
  float fake_lat = 55.6761;  // Example: Copenhagen latitude (to be deleted)
  float fake_lon =  12.5683;  // Example: Copenhagen longitude (to be deleted)
 
  counter = 0;

  send_gps(fake_lat, fake_lon); //sending latitude and longitude
}


void setupAccelerometer() {
  Serial.println("Setting up accelerometer...");
  // TODO: Initialize accelerometer if needed
}


