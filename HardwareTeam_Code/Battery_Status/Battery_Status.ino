// Define pins and stuff
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define SLEEP_BUTTON 0
#define VOLTAGE_PIN 4
#define STATUS_BUTTON 5
// Button setup:
// side1: 3.3V -- resistor -- button -- GPIO
// side2: GND

// CONSTANTS FOR THE BATTERY STATUS CALCULATION //
const float R1 = 14700.0;          // 14.7k
const float R2 = 20000.0;          // 20k
const float ADC_CORRECTION = 0.8;  // Adjust based on testing

// VARIABLES FOR INTERRUPT BUTTONS //
volatile bool statusButtonPressed = false;
volatile bool sleepButtonPressed = false;

void IRAM_ATTR handleStatusButton() {
  statusButtonPressed = true;
}

// void IRAM_ATTR handleSleepButton() {
//   sleepButtonPressed = true;
// }

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_BUTTON, INPUT_PULLUP);
  while (!Serial) {
    delay(10);
  }
  analogReadResolution(12); // Set to 12 bits (0-4095)
  analogSetAttenuation(ADC_11db); // Default is 11db (0-3.3V)
  attachInterrupt(digitalPinToInterrupt(STATUS_BUTTON), handleStatusButton, FALLING);
  // attachInterrupt(digitalPinToInterrupt(SLEEP_BUTTON), handleSleepButton, FALLING);

  // esp_deep_sleep_enable_gpio_wakeup(BUTTON_PIN_BITMASK(SLEEP_BUTTON), ESP_GPIO_WAKEUP_GPIO_LOW);
}

void loop() {
  if (statusButtonPressed) {
    delay(50);
    statusButtonPressed = false;
    readBattery();
    while (digitalRead(STATUS_BUTTON) == LOW);
    delay(10);
  }
  // if (sleepButtonPressed) {
  //   goToSleep();
  // }
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

// void goToSleep() {
//   Serial.print("Going to sleep in 5 seconds");
//   for(int i=0; i<5; i++){
//     delay(1000);
//     Serial.print(".");
//   }
//   Serial.println("");
//   Serial.println("Goodnight!");
//   esp_deep_sleep_start();  // Go to sleep on double-click
// }