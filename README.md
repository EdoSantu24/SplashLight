# BikeProject

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
- 35(BEEPER) & blink = 2 
- 4(H) = (analog read) (JUST BLINKS WHEN TURN TO 5 - FOR AN INSTANT)
- 5(S) = 5,
- 6(S) = 6
- 7(S) = 7
- ??? = 8 = do not touch.
- 12 = ??? (NOTHING)
- 13 = ???
- 14 = ???
- 15(TURN_ON) = 0
- 16(LORA_CONTINGENT CTS) = 1
- 17 == ??? (no)
- 18 = ?
- 19(LORA_CONTINGENT RTS) = 18 (+ analog write)
- 20(LORA_CONTINGENT RST) = 19  (+ analog write) 
- ??? = 20 = (or do not touch - DO NOT TOUCH)
- ??? = 21 = do not touch
- 21 = ?
- 26 = ???
- 0 = ??? = TOUCH NOT!!!!!!!! | WORKING (unknown... oh boot... nvm)


