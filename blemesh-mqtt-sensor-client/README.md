| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- | -------- |

# BLE Mesh MQTT Sensor Client Bridge

This project implements a Bluetooth Low Energy (BLE) Mesh Sensor Client that bridges sensor data from BLE Mesh networks to MQTT brokers. It allows for remote monitoring of sensor data from BLE Mesh devices through standard MQTT protocols.

## Overview

The BLE Mesh MQTT Sensor Client acts as both a BLE Mesh client and an MQTT client. It:

* Discovers and provisions BLE Mesh devices with sensor capabilities
* Periodically requests sensor data from these devices
* Publishes the sensor data to configured MQTT topics
* Creates a seamless bridge between your BLE Mesh sensor network and MQTT-based IoT platforms

## Features

* **BLE Mesh Provisioning**: Automatically discovers and provisions unpaired BLE Mesh devices
* **Sensor Data Collection**: Periodically polls sensor data from BLE Mesh sensor servers
* **MQTT Publishing**: Forwards sensor readings to MQTT topics for remote monitoring
* **MQTT v5.0 Support**: Uses advanced features like user properties, topic aliases, and more
* **Configurable Polling Interval**: Adjustable timing for sensor data collection
* **Reliable Communication**: Implements error handling and retry mechanisms

## Requirements

### Hardware
* ESP32 development board (ESP32, ESP32-C3, or other supported variants)
* USB cable for programming/power
* BLE Mesh devices with Sensor Server model implementation

### Software
* ESP-IDF v4.4 or later
* MQTT broker (e.g., Eclipse Mosquitto, AWS IoT Core, etc.)

## Setup Instructions

### 1. Configure the Project

Key configuration options:

* **Example Connection Configuration**: Configure Wi-Fi connection details
* **Broker URL**: Set your MQTT broker URL
* **BLE Mesh Configuration**: Default settings should work for most applications

### 2. Build and Flash

```
idf.py -p PORT flash monitor
```

Replace PORT with your serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux).

## Usage

Once running, the client will:

1. Connect to Wi-Fi using the configured credentials
2. Start BLE Mesh provisioning to discover compatible sensor devices
3. Connect to the configured MQTT broker
4. Periodically (every 3 seconds) request sensor data from provisioned devices
5. Publish the sensor data to the MQTT topic `/topic/test`

The sensor data is published in a format that includes:
* Sensor type identifier
* Sensor reading value
* Optional metadata (depending on the sensor type)

## Project Structure

* **BLE Mesh Client**: Handles device provisioning and sensor data requests
* **MQTT Client**: Manages broker connection and publishing of sensor data
* **Main Application**: Coordinates the two subsystems and handles the periodic data collection

## Troubleshooting

* **Wi-Fi Connection Issues**: Verify your network credentials in the configuration
* **MQTT Connection Problems**: Check broker URL and ensure network connectivity
* **BLE Mesh Provisioning Failures**: Ensure devices are advertising and within range
* **No Sensor Data**: Verify that the provisioned devices implement the Sensor Server model

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

Based on ESP-IDF BLE Mesh examples and MQTT client examples from Espressif.
