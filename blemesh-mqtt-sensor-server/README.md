| Supported Targets | ESP32 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | --------- | -------- | -------- |

# BLE Mesh Sensor Server with DHT22

This project implements a Bluetooth Low Energy (BLE) Mesh Sensor Server for ESP32 devices with a DHT22 temperature and humidity sensor. It allows temperature and humidity data to be collected and shared through a BLE Mesh network.

## Overview

The BLE Mesh Sensor Server:

- Implements the Sensor Server model from the BLE Mesh specification
- Collects temperature and humidity data from a DHT22 sensor
- Makes sensor data available to BLE Mesh clients (like the BLE Mesh MQTT Sensor Client)
- Can be provisioned wirelessly into a BLE Mesh network
- Supports both Advertising (PB-ADV) and GATT (PB-GATT) provisioning bearers

## Features

- **BLE Mesh Integration**: Full implementation of BLE Mesh node functionality
- **Sensor Model Support**: Implements the Sensor Server model for standardized sensor data reporting
- **Dual Sensor Values**: Reports both temperature and humidity from the DHT22 sensor
- **Auto Reporting**: Updates sensor values periodically
- **Low Power**: Designed for energy-efficient operation
- **Arduino + ESP-IDF Integration**: Uses Arduino as an ESP-IDF component for simplified sensor interfacing

## Requirements

### Hardware
- ESP32 development board (ESP32, ESP32-C3, ESP32-S3, or other supported variants)
- DHT22 temperature and humidity sensor
- Connecting wires
- Power supply (USB or battery)

### Software
- ESP-IDF v4.4 or later
- Arduino as an ESP-IDF component
- BLE Mesh libraries (included in ESP-IDF)
- DHT sensor library

## Hardware Setup

Connect the DHT22 sensor to the ESP32:
- VCC to 3.3V
- GND to GND
- DATA to GPIO4 (default, can be changed in code)

## Software Setup

1. **Configure the Project**
    Key configuration options:
    - Board Selection: Choose your ESP32 development board under "Example Configuration"
    - DHT Pin: Default is GPIO4, can be modified in the code if needed

2. **Build and Flash**
    Replace PORT with your device's serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux).

## Usage

### Provisioning
The device will advertise itself as an unprovisioned BLE Mesh device. Use a BLE Mesh provisioner (like the companion BLE Mesh MQTT Sensor Client) to provision the device into your mesh network.

### Sensor Data Access
Once provisioned, the device will:
- Periodically read temperature and humidity values from the DHT22 sensor
- Make this data available through the BLE Mesh Sensor Server model
- Respond to Sensor Get messages from clients
- Report humidity on Sensor Property ID 0x0056
- Report temperature on Sensor Property ID 0x005B

## Monitoring Sensor Data

The sensor data can be read by:
- BLE Mesh Sensor Client devices
- The BLE Mesh MQTT Sensor Client from this project, which bridges sensor data to MQTT

## Troubleshooting

- **Sensor Reading Issues**: Verify DHT22 connections and ensure the sensor is working properly
- **Provisioning Problems**: Check that the device is advertising and the provisioner is properly configured
- **Communication Errors**: Ensure devices are within BLE range and the mesh network is correctly set up

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

Based on ESP-IDF BLE Mesh examples and Arduino DHT sensor libraries.
