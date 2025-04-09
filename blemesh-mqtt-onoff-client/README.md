# BLE Mesh MQTT On/Off Client Bridge

This project implements a bridge between MQTT and Bluetooth Low Energy (BLE) Mesh networks for the Generic On/Off model. It allows IoT applications to control BLE Mesh devices through standard MQTT protocols.

## Overview

The BLE Mesh MQTT On/Off Client acts as both a BLE Mesh provisioner/client and an MQTT client. It:

- Discovers and provisions BLE Mesh devices (servers)
- Subscribes to an MQTT topic to receive commands
- Forwards MQTT commands to BLE Mesh devices as Generic On/Off commands
- Measures and reports round-trip latency

## Features

- **BLE Mesh Provisioning**: Automatically discovers and provisions unpaired BLE Mesh devices
- **MQTT Integration**: Connects to MQTT brokers with TLS security
- **Command Bridging**: Translates MQTT messages to BLE Mesh on/off commands
- **Latency Measurement**: Reports round-trip time for performance analysis
- **MQTT v5.0 Support**: Uses advanced MQTT protocol features including user properties
- **Resilient Communication**: Includes timeout handling and retry mechanisms

## Requirements

### Hardware
- ESP32 development board (ESP32, ESP32-S3, or other supported variants)
- USB cable for programming/power
- BLE Mesh devices with Generic On/Off Server model

### Software
- ESP-IDF v4.4 or later
- MQTT broker (e.g., Eclipse Mosquitto, AWS IoT Core, etc.)

## Setup Instructions

### 1. Configure the Project
Key configuration options:

- Example Configuration → Board selection: Select your ESP32 development board
- Example Configuration → Broker URL: Set your MQTT broker URL

### 2. Build and Flash
Replace PORT with your serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux).

## Usage

### MQTT Communication
Once running, the client will:

1. Start BLE Mesh provisioning to discover compatible devices
2. Connect to the configured MQTT broker
3. Subscribe to the `/CSC2106/state` topic
4. Listen for messages on this topic to control BLE Mesh devices

When a message is received on the MQTT topic, the client will send a Generic On/Off command to all provisioned BLE Mesh devices.

### Device Provisioning
The client automatically provisions BLE Mesh devices whose device UUID starts with `0xdd 0xdd`. Configure your BLE Mesh servers accordingly.

## Project Structure

- **BLE Mesh Client**: Handles device provisioning, configuration, and command forwarding
- **MQTT Client**: Manages broker connection and message processing
- **Integration Layer**: Connects the two systems to create a unified bridge

## Troubleshooting

- **Provisioning Issues**: Ensure BLE Mesh servers are advertising with UUID starting with `0xdd 0xdd`
- **MQTT Connection Problems**: Check broker URL and TLS certificate
- **Communication Failures**: Verify BLE Mesh devices are within range

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

Based on ESP-IDF BLE Mesh examples and MQTT client examples from Espressif.