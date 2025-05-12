/**
 * @file LoRaWAN_code.ino
 * @brief LoRaWAN communication logic for a smart bicycle light device.
 *
 * This code manages communication between a smart bike light (based on Heltec ESP32 + LoRa module)
 * and The Things Network (TTN) using the LoRaWAN protocol. The device transmits status data such as
 * LED state, battery level, and operation mode periodically. GPS coordinates are sent only upon request.
 *
 * Initial startup messages include multiple transmissions of LED state, mode, and battery level to
 * ensure delivery and proper initialization on the backend. Once initialized, LED and battery
 * statuses are sent cyclically at different intervals. Commands received via downlink allow
 * remote control of device modes or trigger GPS location transmission.
 */

#include "LoRaWan_APP.h"

/* OTAA keys */
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC8 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x99 };
uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x45 };

/* ABP keys (not used) */
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/* Function declarations */
void active_mode();
void parking_mode();
void storage_mode();
void request_gps();

/* LoRaWAN configuration */
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
uint32_t appTxDutyCycle = 15000; // Transmission every 30 seconds
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

/* Startup sequence flags and counters */
bool firstJoinDone = false;
uint8_t startupMessageStep = 0;

/* Device mode and state flags */
bool active = false;
bool parked = true;

/* LED control */
#define LED_PIN 19
bool ledState = false;

/* GPS request flag */
bool gpsRequestPending = false;

/* Transmission control */
int counter = 1;
unsigned long lastLedToggle = 0;
unsigned long lastTx = 0;
const unsigned long toggleInterval = 60000;   // LED toggle every 60s (in this example the led automatically change every 60s)
const unsigned long txInterval = 15000;       // Data TX every 15s

/**
 * @brief Prepares the application payload for regular transmissions.
 * Alternates between sending LED state and battery level.
 */
static void prepareTxFrame(uint8_t port) {
  if (counter <= 3) {
    appDataSize = 1;
    appData[0] = ledState ? 0x0B : 0x0C;
    counter++;
  } else {
    appDataSize = 1;
    appData[0] = readBattery();
    counter = 0;
  }
}

/**
 * @brief Handles downlink messages received from the network server.
 * Recognizes and processes remote commands for mode switching or GPS request.
 */
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  if (mcpsIndication->RxData && mcpsIndication->BufferSize > 0) {
    Serial.print("Downlink received: ");
    for (int i = 0; i < mcpsIndication->BufferSize; i++) {
      Serial.printf("0x%02X ", mcpsIndication->Buffer[i]);
    }
    Serial.println();

    uint8_t command = mcpsIndication->Buffer[0];

    switch (command) {
      case 0x02:
        Serial.println("Command: active_mode");
        active_mode();
        break;
      case 0x03:
        Serial.println("Command: parking_mode");
        parking_mode();
        break;
      case 0x04:
        Serial.println("Command: storage_mode");
        storage_mode();
        break;
      case 0x05:
        Serial.println("Command: request_gps");
        request_gps();
        break;
      default:
        Serial.printf("Unknown command: 0x%02X\n", command);
        break;
    }
  }
}

/**
 * @brief Initial device setup.
 */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
}

/**
 * @brief Main device loop. Manages state transitions, message sending, LED toggling, and sleep cycles.
 */
void loop() {
  unsigned long currentMillis = millis();

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3); // Set data rate
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      firstJoinDone = true;
      startupMessageStep = 0;
      break;

    case DEVICE_STATE_SEND:
      // Initial startup sequence: send LED, mode, battery multiple times
      if (firstJoinDone && startupMessageStep < 6) {
        switch (startupMessageStep) {
          case 0:
          case 1:
            appDataSize = 1;
            appData[0] = ledState ? 0x0B : 0x0C;
            break;
          case 2:
          case 3:
            appDataSize = 1;
            appData[0] = req_mod();
            break;
          case 4:
          case 5:
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
        // Regular periodic transmission
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
      }
      break;

    case DEVICE_STATE_CYCLE:
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
      // Toggle LED periodically
      if (currentMillis - lastLedToggle >= toggleInterval) {
        lastLedToggle = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        Serial.print("LED toggled: ");
        Serial.println(ledState ? "ON" : "OFF");
      }

      // Trigger next transmission
      if (currentMillis - lastTx >= txInterval) {
        lastTx = currentMillis;
        deviceState = DEVICE_STATE_SEND;
      }

      // Enter LoRaWAN sleep mode
      LoRaWAN.sleep(loraWanClass);
      break;

    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}

/**
 * @brief Simulated battery level reading.
 * @return Mock battery value.
 */
float readBattery() {
  return 0x5A;
}

/**
 * @brief Activates "active" operation mode.
 */
void active_mode() {
  Serial.println("Active mode activated");
}

/**
 * @brief Activates "parking" operation mode.
 */
void parking_mode() {
  Serial.println("Parking mode activated");
}

/**
 * @brief Activates "storage" operation mode.
 */
void storage_mode() {
  Serial.println("Storage mode activated");
}

/**
 * @brief Returns current mode code as float: 0x02=active, 0x03=parking, 0x04=storage.
 */
float req_mod() {
  if (active) return 0x02;
  else if (parked) return 0x03;
  else return 0x04;
}

/**
 * @brief Sends GPS coordinates encoded in IEEE 754 format to TTN.
 * @param latitude Latitude value (float)
 * @param longitude Longitude value (float)
 */
void send_gps(float latitude, float longitude) {
  Serial.println("Sending GPS coordinates to TTN...");

  uint8_t *latPtr = (uint8_t*)&latitude;
  uint8_t *lonPtr = (uint8_t*)&longitude;

  appDataSize = 8;
  for (int i = 0; i < 4; i++) appData[i] = latPtr[i];
  for (int i = 0; i < 4; i++) appData[i + 4] = lonPtr[i];

  LoRaWAN.send();
}

/**
 * @brief Simulates a GPS request by sending hardcoded coordinates (e.g., Copenhagen).
 */
void request_gps() {
  Serial.println("GPS request triggered. Getting fake coordinates...");

  //fake coordinates, to change with the GNSS ones
  float fake_lat = 55.6761;  // Copenhagen latitude hardcoded value 
  float fake_lon = 12.5683;  // Copenhagen longitude hardcoded value 

  counter = 0; // Reset counter to prioritize next transmissions

  send_gps(fake_lat, fake_lon);
}


