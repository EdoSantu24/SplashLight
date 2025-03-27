## Sensor Team

### Test environment: LCD

- **Database for data storage** (route history, light status, theft alarm): 
  - Technologies: Node.js and PostgreSQL (to decide: Explore alternative solutions)


### GY-521: MPU-6050 Accelerometer

- **  Accelerometer**

### Photoresistor

- Must determine the proper threshold to be considered "lit" and "dark"

### ion battery charger module 03962a


- Must determine the proper threshold to be considered "lit" and "dark"

### do not burn objects
- So if a device requires 5 V or any other volt above 3.3 V, then the ESP32 might be damaged when it tries to supply the device. in this case you would need another, external power source

### deep-sleep mode
- determine which sensors must be on during this, and which sensors should manage to wake it up
- develop deep sleep handling of sensors - and how they may wake the sensor up.

### Operational modes



---
