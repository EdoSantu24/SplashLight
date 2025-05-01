// Libraries
#include <WiFi.h>
#include <Wire.h>
#include <pitches.h>
// #include <LoRa.h> // LoRa libraries
// #include <Adafruit_Sensor.h> // Example for accelerometer
// #include <Adafruit_LIS3DH.h> // Example accelerometer

// Define pins
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER
#define VOLTAGE_PIN 4
#define LED_PIN 5
#define AUDIO_PIN 18
#define ACCELO_PIN 19
#define LIGHT_SENS_PIN 34
// #define WIFI_PIN 21 // optional if needed to enable WiFi manually

// Constants for battery status calculation
const float R1 = 14700.0;          // 14.7kΩ
const float R2 = 20000.0;          // 20kΩ
const float ADC_CORRECTION = 0.8;  // Calibration factor

// Variables
bool active = false;
bool parked = true;

void setup() {
  // Set up Serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  analogReadResolution(12); // 12-bit ADC
  analogSetAttenuation(ADC_11db); // 0-3.3V range

  // Set up pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(AUDIO_PIN, OUTPUT);
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

void loop() {
  // Empty main loop; Modes manage themselves
}

void activeMode() {
  Serial.println("Entering Active Mode");

  digitalWrite(LED_PIN, LOW); // LED OFF initially
  // Turn on WiFi and LoRa modules
  // Turn on accelerometer
  // Turn on light detection
  setupWiFi();
  setupLoRa();

  active = true;
  parked = false;

  while (active) {
    float battery = readBattery();

    if (battery <= 20.0) {
      // Critical battery warning
      static unsigned long lastWarningAct = 0;
      if (millis() - lastWarningAct > 30 * 1000) { // every 30 sec
        playCriticalBatteryWarning();
        sendBatteryWarning();
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

  digitalWrite(LED_PIN, LOW);
  // turn on accelerometer & wireless comm. 
  // TODO: Turn off unnecessary components
  // Assuming you can turn off Light sensor etc.

  active = false;
  parked = true;

  while (parked) {
    float battery = readBattery();
    if (battery <= 20.0) {
      static unsigned long lastWarningPark = 0;
      if (millis() - lastWarningPark > 15 * 60 * 1000) { // every 15 min
        sendBatteryWarning(); // Send warning through LoRa
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

  digitalWrite(LED_PIN, LOW);
  //turn off light detection & accelerometer
  //turn on wifi & LoRa
  
  active = false;
  parked = false;

  while (!active && !parked) {
    float battery = readBattery();
    if (battery <= 20.0) {
      static unsigned long lastWarningStore = 0;
      if (millis() - lastWarningStore > 60 * 60 * 1000) { // every hour
        sendBatteryWarning(); // Send warning through LoRa
        lastWarningStore = millis();
      }
    }
    checkIncomingMessage();
    delay(1000);
  }
}


// Helper Functions

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

  if (batteryVoltage >= 4.2) Serial.println("Battery Full!");
  if (batteryVoltage <= 3.0) Serial.println("Charge soon!");

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
void sendBatteryWarning() {
  Serial.println("Sending low battery warning...");
  // TODO: Send a battery warning over WiFi / LoRa
}

bool movementDetected() {
  // Simple dummy example: Read accelerometer pin
  int movement = digitalRead(ACCELO_PIN);
  return movement == HIGH; // adjust depending on your accelerometer logic
}

void checkIncomingMessage() {
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
      sendBatteryUpdate();
    } else if (incoming == "appLocationUpdate") {
      sendLocationUpdate();
    }
  }
}

bool checkLight(){
 int lightLevel = analogRead(LIGHT_SENS_PIN);
  Serial.printf("Light sensor reading: %d\n", lightLevel);
  return lightLevel > 2000; // tune threshold
}

void sendBatteryUpdate(){
}

void sendLocationUpdate(){
}

void setupWiFi() {
  Serial.println("Setting up WiFi...");
  // TODO: Connect to WiFi network
}

void setupLoRa() {
  Serial.println("Setting up LoRa...");
  // TODO: Initialize LoRa module
}

void setupAccelerometer() {
  Serial.println("Setting up accelerometer...");
  // TODO: Initialize accelerometer if needed
}
