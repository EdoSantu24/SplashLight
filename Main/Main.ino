// Libraries //

#include <WiFi.h>
#include <Wire.h>
#include <pitches.h>
#include <../Sensor/SENSOR_TEAM_Initial_Worksheet/accelerometer.h>
#include <../Sensor/SENSOR_TEAM_Initial_Worksheet/photoresistor.h>
#include <../Sensor/SENSOR_TEAM_Initial_Worksheet/accelerometer.ino>
#include <../Sensor/SENSOR_TEAM_Initial_Worksheet/photoresistor.ino>

#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"

// ###### LoRaWAN Setups ##### //

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
bool joinedLoRa = false;


bool ledState = false;
/* GPS request flag */
bool gpsRequestPending = false;
/* Transmission control */
int counter = 1;

unsigned long lastTx = 0;
const unsigned long txInterval = 15000;       // Data TX every 30s

// GPS variables
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ##### END of LoraWAN setups ###### //

// Define pins
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER
#define SDA_PIN 2
#define SCL_PIN 9
#define LED_PIN 19
// Photoresistor pin 0 is also used

// NOT USED PINS
#define SOUND_PIN 45
#define VOLTAGE_PIN 4
#define GPS_PIN1 46
#define GPS_PIN2 47
// #define WIFI_PIN 21 // optional if needed to enable WiFi manually

// Constants for battery status calculation
const float R1 = 14700.0;          // 14.7kΩ
const float R2 = 20000.0;          // 20kΩ
const float ADC_CORRECTION = 0.8;  // Calibration factor

// Variables
bool active = false;
bool parked = true;

volatile int current_mode = 0; // 0 = ACTIVE MODE | 1 = PARKING MODE | 2 = STORAGE MODE
volatile bool interrupted = false; // When interrupted by downlink this variable changes

/**
 * Initial device setup.
**/
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
  // pinMode(SOUND_PIN, OUTPUT);

  // Set up GPS
  gpsSerial.begin(115200, SERIAL_8N1, GPS_PIN1, GPS_PIN2);

  // I2C
  Wire.begin(SDA_PIN,SCL_PIN);
  // Accelerometer setup
  setupAccelerometer();
  setAccelerometerThresholds(30, 10000);

  // Photoresistor
  setupPhotoresistor();
  setPhotoresistorThreshold(500);

  // Setup Button if enough pins are available
  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  current_mode = 1; // Start in parking mode
}

/**
 * @brief Main device loop. Manages state transitions to different modes,
 * which are stored in the 'current_mode' variable.
 * 0 : Active Mode \\ 1 : Parking Mode \\ 2 : Storage Mode
**/
void loop() {
  while (!joinedLoRa) {
    checkLoRa();
  }
  switch (current_mode) {
    case 0:
      current_mode = activeMode();
      break;
    case 1:
      current_mode = parkingMode();
      break;
    case 2:
      current_mode = storageMode();
      break;
  }
}

/**
 * @brief Active Mode, turns LED on/off depending on light detected, turns accelerometer on. 
 * Plays a buzzer when battery level is critical. Will switch to Parking Mode if no movement has been detected
**/
int activeMode() {
  // Initialize Active Mode //
  Serial.println("Entering Active Mode");

  digitalWrite(LED_PIN, LOW); // initially turning off LED
  ledState = false;
  setupAccelerometer();

  // Send current mode info via uplink
  send_mode();

  active = true;
  parked = false;

  // State tracking variables
  bool movement = true;
  bool countingDown = false;
  unsigned long countdownStart = 0;
  unsigned long lastMovementCheck = 0;

  // Main loop in Active Mode //
  while (active) {

    // Check if it was interrupted by downlink
    if (interrupted){
      interrupted = false;
      return current_mode;
    }

    // Always check LoRa
    checkLoRa();

    // Critical battery warning
    float battery = readBattery();
    static unsigned long lastWarningAct = 0;
    if (battery <= 20.0 && millis() - lastWarningAct > 60 * 1000) { 
      playCriticalBatteryWarning();
      lastWarningAct = millis();
    }

    // Light detection
    bool lightDetected = checkLightThreshold(-30); //the argument is not used
    digitalWrite(LED_PIN, lightDetected ? LOW : HIGH);
    ledState = !lightDetected;

    // Movement detection
    if (millis() - lastMovementCheck >= 2000) { // Check every 2 seconds
      lastMovementCheck = millis();
      movement = isMovingNow();

      if (!movement && !countingDown) {
        Serial.println("No movement detected. Starting 15s countdown.");
        countdownStart = millis();
        countingDown = true;
      } else if (movement && countingDown) {
        Serial.println("Movement resumed. Cancelling countdown.");
        countingDown = false;
      }
    }
    
    // Handle countdown
    if (countingDown && (millis() - countdownStart >= 15000)) { // after 15 sec
      Serial.println("No movement after 15 seconds. Switching to Parking Mode.");
      current_mode = 1;
      return current_mode; // Switch mode
    }
  }
  return current_mode;
}

/**
* @brief Parking Mode, turns LED off and Accelerometer on. Switches to active mode if movement is detected. 
*/
int parkingMode() {
  // Initialize Parking Mode //
  Serial.println("Entering Parking Mode");

  digitalWrite(LED_PIN, LOW); // initially turning off LED
  ledState = false;
  setupAccelerometer();

  // Send current mode info via uplink
  send_mode();

  active = false;
  parked = true;

  unsigned long lastMovementCheck = 0;

  // Main loop in Parking Mode //
  while (parked) {  

    // Check if it was interrupted by downlink message
    if (interrupted){
      interrupted = false;
      return current_mode;
    }

    checkLoRa();

    // Check for movement every 2 seconds
    if (millis() - lastMovementCheck >= 2000) {
      lastMovementCheck = millis();
      if (isMovingNow()) {
        Serial.println("Movement detected. Switching to Active Mode.");
        current_mode = 0;
        return current_mode;
      }
    }
  }
  return current_mode;
}

/**
* @brief Storage Mode, turns LED and Accelerometer off. ONLY switches 
* modes if specified from app. 
*/
int storageMode() {
  // Initialize Storage Mode //
  Serial.println("Entering Storage Mode");

  digitalWrite(LED_PIN, LOW); //turning off LED
  ledState = false;
  turnOffAccelerometer();

  // Send current mode info via uplink (even though it is kinda unnecessary)
  send_mode();
  
  active = false;
  parked = false;

  // Main loop in Storage Mode //
  while (!active && !parked) {
    if (interrupted){
      interrupted = false;
      return current_mode;
    }
    checkLoRa();
  }
  return current_mode;
}

// #################################### //
// ########## LORA FUNCTIONS ########## //
// #################################### //

/**
 * @brief Handles downlink messages received from the network server.
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

    interrupted = true;

    switch (command) {
      case 0x02:
        Serial.println("Command recieved: activeMode");
        current_mode = 0; // activeMode
        break;

      case 0x03:
        Serial.println("Command recieved: parkingMode");
        current_mode = 1; // parkingMode
        break;

      case 0x04:
        Serial.println("Command recieved: storageMode");
        current_mode = 2; // storageMode
        interrupted = true;
        break;

      case 0x05:
        Serial.println("Command recieved: request_gps");
        request_gps();
        break;

      default:
        Serial.print("Command unknown: 0x");
        Serial.println(command, HEX);
        interrupted = false;
        break;
    }
  }
}

/**
 * @brief Function for sending LoRa messages. Also the
 * one used for initializing communication to LoRa
 */
void checkLoRa(){
  unsigned long currentMillis = millis();
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      Serial.println("INIT DONE");
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      // Serial.println("JOIN done?");
      firstJoinDone = true; // When joined is connected
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
              joinedLoRa = true;
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

// ######################################### //
// ######### LORA HELPER FUNCTIONS ######### //
// ######################################### //

/**
 * helper function to make the board able to send data of the current mode via uplink 
 */
void send_mode() {
  appDataSize = 1;
  appData[0] = req_mod();
  LoRaWAN.send();
}

/**
 * Function that translates the current_mode value into hexadecimals, 
 * ready to be sent through uplink via LoRa.
 */
float req_mod() {
  if (current_mode == 0) {
    return 0x02; //hex 02 is active
  }
  else if (current_mode == 1) {
    return 0x03; //hex 03 is parked
  } 
  else {
    return 0x04; //hex 04 is storage
  }
}

/**
 * @brief Sends GPS coordinates encoded in IEEE 754 format to TTN.
 * @param latitude Latitude value (float)
 * @param longitude Longitude value (float)
**/
void request_gps() {
  Serial.println("GPS request triggered. Sending coordinates...");

  float fake_lat = 55.6761;
  float fake_lon = 12.5683;

  if (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  // Print GPS location if available
  if (gps.location.isUpdated()) {
    fake_lat = gps.location.lat();
    fake_lon = gps.location.lng();
  }
  // this hardcoded latitude and longitude need to be replaced with the actual latitude and longitude retrieved by the GNSS
  counter = 0;

  send_gps(fake_lat, fake_lon); //sending latitude and longitude
}

/**
 * Function that Encode 2 floats (lat, lon) into 8 bytes (4 bytes each, IEEE 754),
 * and sends the data through LoRa with the LoRaWAN.send() command.
 */
void send_gps(float latitude, float longitude) {

  uint8_t *latPtr = (uint8_t*) &latitude;
  uint8_t *lonPtr = (uint8_t*) &longitude;

  appDataSize = 8;
  for (int i = 0; i < 4; i++) appData[i] = latPtr[i];
  for (int i = 0; i < 4; i++) appData[i + 4] = lonPtr[i];

  LoRaWAN.send();
}

// ########################################## //
// ######### OTHER HELPER FUNCTIONS ######### //
// ########################################## //

/**
 * Reads the battery charge on the VOLTAGE_PIN. 
 * Returns the battery charge in percent as a float.
 */
float readBattery() {
  return 60.0;
  int raw = analogRead(VOLTAGE_PIN);
  float vPin = (raw * ADC_CORRECTION * 3.3 / 4095.0);
  float batteryVoltage = vPin * (R1 + R2) / R2;
  float batteryCharge = mapBatteryPercentage(batteryVoltage);
  Serial.print("Charge: ");
  Serial.println(batteryCharge);
  return batteryCharge;
}

/**
 * Simple function that maps voltage charge to a specific percentage as a float.
 */
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

/**
 * Function that uses the SOUND_PIN to play a short beep sound.
 * To be used as a battery warning.
 */
void playCriticalBatteryWarning() {
  Serial.println("Critical battery! Playing audio warning.");
  int melody = NOTE_E6;
  int duration = 250;
  tone(SOUND_PIN, melody, duration);
  delay(250);
  tone(SOUND_PIN, melody, duration);
}
