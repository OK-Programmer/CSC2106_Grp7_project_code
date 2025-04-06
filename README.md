# BLE Mesh MQTT Bridge Project

This repository contains the code for our IoT project using BLE Mesh and MQTT for sensor data collection and control. The project creates a bridge between BLE Mesh devices and MQTT brokers, allowing for remote monitoring and control.

## System Architecture

The system consists of four main components:

### [blemesh-mqtt-onoff-client](./blemesh-mqtt-onoff-client/)
BLE Mesh client that connects to the onoff server, allowing control of LED states via MQTT. This client acts as a bridge between MQTT and the BLE Mesh network.

### [blemesh-mqtt-onoff-server](./blemesh-mqtt-onoff-server/)
BLE Mesh server that controls LED states based on commands received from the client through the mesh network. Can be provisioned to respond to on/off commands.

### [blemesh-mqtt-sensor-client](./blemesh-mqtt-sensor-client/)
BLE Mesh client that connects to the sensor server, reads sensor data from the mesh network, and publishes it to MQTT topics for remote monitoring.

### [blemesh-mqtt-sensor-server](./blemesh-mqtt-sensor-server/)
BLE Mesh server with temperature and humidity sensors. Collects sensor data and makes it available on the mesh network for clients to read.

## Hardware Requirements

- ESP32 development boards (ESP32, ESP32-S3, or other compatible variants)
- Sensors (for sensor server nodes)
- Actuators (for on/off server nodes)
- USB cables for programming
- Power supply

## Software Dependencies

- ESP-IDF v5.1 (Espressif IoT Development Framework)
- Arduino as an ESP-IDF component
- MQTT library
- BLE Mesh components

## Building and Flashing

### Prerequisites
- Install ESP-IDF according to official instructions
- Set up the ESP-IDF environment variables

### Configuration
Each component can be configured using the ESP-IDF menuconfig system:
```
idf.py menuconfig
```

Key configurations include:
- WiFi credentials
- MQTT broker details
- BLE Mesh parameters

### Building and Flashing
For each component:
```
idf.py build
idf.py -p PORT flash
```
Replace PORT with your device's serial port (e.g., /dev/ttyUSB0 on Linux, COM3 on Windows).

### Monitor Output
```
idf.py -p PORT monitor
```

## MQTT Topics

- `/CSC2106/state` - For on/off status updates
- `/topic/test` - For sensor data (temperature and humidity)

## Troubleshooting

- **Connection Issues**: Ensure WiFi credentials and MQTT broker details are correctly configured
- **Mesh Provisioning Failures**: Check BLE signal strength between devices
- **Compilation Errors**: Verify ESP-IDF version compatibility

## License

This project is licensed under MIT License.