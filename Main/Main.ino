// libraries


// Define pins and stuff
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define VOLTAGE_PIN 4
#define LED_PIN
#define AUDIO_PIN
#define ACCELO_PIN
#define LIGHT_SENS_PIN
#define WIFI_PIN

// Button setup:
// side1: 3.3V -- resistor -- button -- GPIO
// side2: GND

// CONSTANTS FOR THE BATTERY STATUS CALCULATION //
const float R1 = 14700.0;          // 14.7k
const float R2 = 20000.0;          // 20k
const float ADC_CORRECTION = 0.8;  // Adjust based on testing

void setup() {
  // Set up Serial //
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  analogReadResolution(12); // Set to 12 bits (0-4095)
  analogSetAttenuation(ADC_11db); // Default is 11db (0-3.3V)

  // Set up LED pin //
  // Set up Audio pin //
  // Set up Accelerometer //
  // Set up Light detection //
  // Set up Wifi module //
  // Set up LoRa //

  // Optional: Set up physical button for switching modes //

  // Go to parkingMode()
}

void activeMode() {
  // INITIALISE //
  // Turn off LED
  // Turn on Accelerometer
  // Turn on WiFi
  // Turn on LoRa
  // Turn on Light detection

  // Send Geolocation
  // Send Battery status

  // ACTIVE-MODE LOOP //
  // If battery is critical -> Send audio + message warning (every 30 sec or so)
  // Check if it detects light -> turn on/off LED
  // Movement detected = yes -> repeat loop
  // Movement detected = no -> Step out of loop

  // Begin timer / loop count
  // COUNTDOWN LOOP //
  // Check if movement detected -> yes = go back to active mode loop
  // Check if message from app:
  //    -> Send and battery status
  //    -> Switch to corresponding mode
  // Check if timer ended -> activate parking mode
  // Delay for 3 sec or so, and repeat loop

}

void parkingMode() {
  // INITIALISE //
  // Turn off LED
  // Turn off Light Detection
  // Turn on Accelerometer
  // Turn on WiFi
  // Turn on LoRa

  // PARKING-MODE LOOP (loops every 1 sec) //
  // If battery is critical -> Message warning (every 15 min)
  // If movement detected -> Go to activeMode()
  // If no message from app -> delay 1 sec, Go back to loop
  // If message from app:
  //    -> "Get status": delay 1 sec, send Geolocation and Battery status + go back to loop
  //    -> "Change mode": go to corresponding mode
}

void storageMode() {
  // INITIALISE //
  // Turn off LED
  // Turn off Light Detection
  // Turn off Accelerometer
  // Turn on WiFi
  // Turn on LoRa

  // PARKING-MODE LOOP (loops every 1 sec) //
  // If battery is critical -> Message warning (every 15 min)
  // If no message from app -> delay 1 sec, Go back to loop
  // If message from app:
  //    -> "Get status": delay 1 sec, send Geolocation and Battery status + go back to loop
  //    -> "Change mode": go to corresponding mode
}


void loop() {
}

void readBattery() {
  int raw = analogRead(VOLTAGE_PIN);
  float vPin = (raw * ADC_CORRECTION * 3.3 / 4095);
  float batteryVoltage = vPin * (R1 + R2) / R2;
  float batteryCharge = mapBatteryPercentage(batteryVoltage);
  Serial.print("Charge: ");
  Serial.print(batteryCharge);
  Serial.print(" | raw: ");
  Serial.print(raw);
  Serial.print(" | Voltage on pin: ");
  Serial.println(vPin);
  if (batteryVoltage >= 4.2) Serial.println("Battery Full!");
  if (batteryVoltage <= 3.0) Serial.println("Charge soon!");
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