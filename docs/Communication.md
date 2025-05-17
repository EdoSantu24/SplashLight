## Communication Team

### Communication Technology: LoRaWAN and TTN

- **HT-CT62 ESP32C3 SX1262 LoRa Nodemodule Bluetooth LoRaWAN Node** (https://docs.heltec.cn/en/node/esp32/ht_ct62/index.html)
- Low latency and low energy consumption
- **TTN** for network coverage in Copenhagen
- Communication between the ESP32C3 is handled by LoRaWAN gateways and exchange of messages can be seen from the TTN dashboard.

---

### Communication Protocol for backend: MQTT

- Communication with the mobile application relies on MQTT protocol.
- Using TTN's MQTT built-in broker and a private HiveMQ Broker.
- Energy efficient and low latency
- **Quality of Service (QoS)** support: Provides flexibility on message delivery
- **Pub/Sub technology**: Enables deep sleep mode on the device to conserve energy
- MQTT broker options: **HiveMq**
---

### Backend: NodeRed and Oracle Cloud VM
- Communication between the two MQTT broker relies on NodeRed installed on a VM hosted on the cloud
- Cloud VM is provided by Oracle Cloud
- **NodeRed** act as bridge between the TTN MQTT broker and the private HiveMQ broker. It also act as a translator for the messages transmitted by the device (hex format) to json format used by the android application.

---

### Frontend: Android Application
- Android app developed on Flutter using Android Studio.
- Provide remote control and monitor features including:
- **LED Control** – Turn the bike light ON or OFF remotely.
- **Battery Status** – View the device’s current battery level.
- **Low Battery Notification** – Receive an alert when the battery drops below 20%.
- **Mode Selection** – Switch between:
  - *Active Mode*
  - *Parking Mode*
  - *Storage Mode*
- **GPS Integration** – Request and view the device’s real-time location.
- **Live Map Display** – See both your position and the light’s position using OpenStreetMap.
- **MQTT Communication** – Communicates with the smart light through MQTT protocol.



---
