#define LED 18  // LED on GPIO4
#define BUTTON 4 // Button on GPIO18

// Sleep mode definitions
#define WAKEUP_GPIO 4
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex

int buttonState;
unsigned long clickInterval = 500;
unsigned long lastPressTime = 0;
int clicks = 0;
volatile bool flag = false;


// In this interrupt, which is activated when 
// clicking on the button, flag is set to true
void IRAM_ATTR buttonISR() {
  flag = true;

  // CODE FOR RESTARTING WHEN DOUBLE CLICKING (only works 50% of the time though)
  // unsigned long currentTime = millis();

  // if (currentTime - lastPressTime > 50) {  // Debounce (ignore very fast repeats)
  //   if (currentTime - lastPressTime < clickInterval) {  // Check if it's a double-click
  //     clicks++;

  //     if (clicks == 2) {
  //       flag = true;
  //     }
  //   } else {
  //       clicks = 1;  // Reset if too much time passed
  //   }
  //   lastPressTime = currentTime;
  // }
}

void setup() {
  Serial.begin(115200);
  // initialize the LED pin as an output:
  pinMode(LED, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, RISING);
  delay(1000);

  ledOn();
  delay(500);
  ledOff();

  // This enables the esp to be awoken by a GPIO - in this case a button
  esp_deep_sleep_enable_gpio_wakeup(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_GPIO_WAKEUP_GPIO_HIGH);
  gpio_pulldown_en((gpio_num_t)WAKEUP_GPIO);
  gpio_pullup_dis((gpio_num_t)WAKEUP_GPIO);

}

void loop() {
  if (flag == true){
    goToSleep();
  }
}

// Helper functions //

// Will first blink, and then proceed to enter deep sleep mode
void goToSleep(){
  ledOn();
  delay(250);
  ledOff();
  delay(250);
  ledOn();
  delay(1000);
  ledOff();

  // since the wakeup button is the same as "sleep button", 
  // we need to wait before the button is not clicked
  while (digitalRead(BUTTON) == HIGH) { 
    delay(50);
  }
  Serial.print("Going to sleep...");
  esp_deep_sleep_start();  // Go to deep sleep
}

// short led functions
void ledOn() {
  digitalWrite(LED, HIGH);
}
void ledOff() {
  digitalWrite(LED, LOW);
}