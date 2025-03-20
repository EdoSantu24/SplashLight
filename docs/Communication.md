## Communication Team

### Communication Technology: LoRaWAN

- **HT-CT62 ESP32C3 SX1262 LoRa Nodemodule Bluetooth LoRaWAN Node**
- Low latency and low energy consumption
- TTN (The Things Network) for network coverage in Denmark
- Bidirectional communication (bike <-> backend app/dashboard IoT)

---

### Communication Protocol: MQTT

- Supports bidirectional communication (user <-> bike light)
- Energy efficient and low latency
- **Quality of Service (QoS)** support: Provides flexibility on message delivery
- **Pub/Sub technology**: Enables deep sleep mode on the device to conserve energy
- Compatible with TTN
- MQTT broker options (to decide: **Mosquitto**?)

---

### Backend

- **Database for data storage** (route history, light status, theft alarm): 
  - Technologies: Node.js and PostgreSQL (to decide: Explore alternative solutions)

---

### Frontend

- **Dashboard for display, control, and monitoring**: 
  - To decide: Custom Android app, **Blynk**, or web dashboard (using **React** or **Flutter**)
  
- **Monitoring Features**:
  - GPS data
  - General statistics: Battery status, route history (e.g., km)
  - **Geofencing Alarm**: Receive a notification if the bike leaves a predefined area
  - **Anti-theft Mode**: Notification if the bike moves without authorization (e.g., identification via phone; if the bike is used without having the phone with you)

- **Control Features**:
  - Turn the light on/off via app
  - Change configurations/settings: Notifications, alarm settings, dark mode, etc.

- **Security** (to be determined?)

---
