# BikeProject
Welcome to the BikeProject repository! This project is a collaborative effort to create a smart bike light system that turns automatically on/off, as well as enhancing safety for cyclists. Below you will find an overview of the project, as well as how to get started with the code and hardware.

## Project Overview
For this project, we have used the ESP32-C3 microcontroller that has a built-in Wi-Fi and LoRa module. This microcontroller allows us to be able to remotely control and monitor the bike light as well as the bike itself. The bike light is designed to be energy-efficient, to extend battery life as much as possible. The code for the project is written in the Arduino programming language, which based on C/C++.
For this smart light project, the following modules are required:
- ESP32-C3 microcontroller
- Battery
- Battery charger
- Voltage regulator
- Accelerometer
- P-MOSFET
- LED light
- Photoresistor
- Buzzer
- Physical button
- Resistors

## Getting Started
To get started, you will need to set up the code as well as the hardware. We will provide you with a step-by-step guide on how to do both of these steps.

### Hardware Setup
To set up the hardware, you will need the components listed in the project overview. You will also need basic tools like breadboard and jumper wires to connect the components together.

### Code Setup
First of all, you need to install the Arduino IDE on your computer. You can download it from the official Arduino website: [Arduino IDE](https://www.arduino.cc/en/software/).
Next, you should install the ESP32 board package in the Arduino IDE. To do this, follow these steps:
1. In the Arduino IDE, go to File -> Preferences.
2. First, set the "Sketchbook location" to a folder containing this project.
3. In the "Additional Board Manager URLs" field, add the following URL: `https://resource.heltec.cn/download/package_heltec_esp32_index.json`
4. In the Library Manager, search for "Heltec" and install the package called "Heltec ESP32 Dev-Boards" by Heltec Automations.
5. Go to Tools -> Board -> Board Manager and select "Wireless Mini Shell".
Now you are ready to run the code in "Main.ino" from the Arduino IDE without any errors.

Remember to get the following libraries:
 - Adafruit_GFX.h

## Required features of Bike Light
- (HT) Based on ESP32 microcontroller.
- (HT) Manual switch-on by button-press.
- (HT) Low power consumption. Focus on battery life.
- (HT) Battery driven & rechargeable.
- (HT) 3D printed enclosure.
- (ST) Capable of auto turn-on light when moved (accelerometer).
- (ST) Equipped with light sensor to auto switch-on at night, if moved (LDR).
- (ST) Auto turn-off if not moved in 30 seconds.
- (ST) Operation modes: *Active mode, Park mode, Storage mode*.
- (ST/CT) Capable of changing operation mode from backend/frontend/app.
- (CT) Capable of sending a geolocation at trip start & stop (GNSS/WiFi-scanning).
- (CT) Equipped with LoraWAN for wireless communication.
- (CT) Capable of transmitting data: Geolocation & battery status.
- (ST) Visual or audible warning of low-battery (<20%).
- (ET) Capable of being field-tested. 

## Teams
- Hardware-Power Team   (HT) Nicklas, Pepe
- Sensor Team           (ST) Julian, Oskar
- Communication Team    (CT) Gea, Edoardo
- Everything Team       (ET) Everyone

## Next time meeting
online or in person? We can communicate via WhatsApp.

## pins to pinmode
pins: which pins on the ESP32 board tha pinmode corresponds to and which can be used:
- [on board pin number]([allocated for]) = [the pinMode() number]
- 0 = ??? = TOUCH NOT!!!!!!!! (boot?
- 4(H) = (analog read) (JUST BLINKS WHEN TURN TO 5 - FOR AN INSTANT)
- 5(S ON/OFF SMART LIGHT) = 5,
- 6(S SDA) = 6
- 7(S SCL) = 7
- ??? = 8 = do not touch.
- 12 = ??? (NOTHING)
- 13 = ??? (NOTHING)
- 14 = ??? (NOTHING)
- 15(Photoresistor_reading) = 0 (ADC)
- 16() = 1 (ADC?)
- 17 = ??? (NOTHING)
- 18 = ??? (NOTHING)
- 19(Turn_off_accelerometer ) = 18 (+ analog write)
- 20(H The bike light) = 19  (+ analog write) 
- ??? = 20 = do not touch 
- ??? = 21 = do not touch
- 21 = ??? (NOTHING)
- 26 = ??? (NOTHING)
- 35(BEEPER) & blink = 2 



