# Smart Bike Light – Android Application

This Flutter-based Android app is part of the **Smart Bike Light** project. The app acts as a monitoring and control interface for a custom smart light device installed on the bike, allowing the user to interact with the light in real-time.

## Features

- **Live LED Status Control**: Turn the smart light ON or OFF directly from the app and see led status.
- **Battery Monitoring**: Displays the current battery level of the device.
- **Low Battery Alert**: Automatically notifies the user when the battery drops below 20%.
- **Mode Selection**: Choose between *Active*, *Parking*, and *Storage* modes.
- **GPS Integration**:
  - Request the device’s current GPS location.
  - Display both the user and light's positions on a map.
- **MQTT Communication**: Uses MQTT to receive live updates and send commands to the device.

## Technologies Used

- **Flutter** – for building the cross-platform UI.
- **MQTT** – for lightweight messaging between the app and the IoT device.
- **Google Fonts** – for enhanced typography.
- **Flutter Map** – for OpenStreetMap integration.
- **Geolocator** – to get the phone’s location.

