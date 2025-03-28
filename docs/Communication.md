## Communication Team

### Communication Technology: LoRaWAN

- **HT-CT62 ESP32C3 SX1262 LoRa Nodemodule Bluetooth LoRaWAN Node**
- Low latency and low energy consumption
- **TTN** (or **Chirpstack**) for network coverage in Denmark: support MQTT broker (HiveMQ)
- Bidirectional communication (light <-> LoreWAN gateways <-> MQTT broker)

---

### Communication Protocol for backend: MQTT

- Supports bidirectional communication (backend app/dashboard IoT <-> MQTT broker)
- Energy efficient and low latency
- **Quality of Service (QoS)** support: Provides flexibility on message delivery
- **Pub/Sub technology**: Enables deep sleep mode on the device to conserve energy
- Compatible with TTN
- MQTT broker options: **HiveMq**
  - Integrating ESP32 with LoRaWAN and HiveMQ MQTT Broker: https://www.hivemq.com/blog/integrating-esp32-lorawan-hivemq-mqtt-broker-advanced-iot/
  - LoRaWAN and MQTT Integration for IoT Application Design:  https://www.hivemq.com/blog/lorawan-and-mqtt-integrations-for-iot-applications-design/
  - Hands-on Guide to LoRaWAN and HiveMQ MQTT Broker Integration for IoT: https://www.hivemq.com/blog/handson-guide-lorawan-hivemq-mqtt-broker-integration-iot/
  - Creating a private MQTT Broker with HiveMQ
  - It has it's own API to retrieve data and send to a database/application (GPS data, battery statistics, route history...)
---

### Backend

- **Database for data storage** (route history, light status, theft alarm): 
  - Technologies: Node.js and PostgreSQL (to decide: Explore alternative solutions)
  - has to be on the cloud?
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
