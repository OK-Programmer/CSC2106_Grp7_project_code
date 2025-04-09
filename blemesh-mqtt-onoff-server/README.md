# BLE Mesh On/Off Server with IR Control

This project implements a Bluetooth Low Energy (BLE) Mesh On/Off Server for ESP32 devices that translates BLE Mesh commands into infrared (IR) signals. It creates a bridge between modern BLE Mesh networks and traditional IR-controlled devices like TVs, air conditioners, or other consumer electronics.

## Overview

The BLE Mesh On/Off Server:

- Implements the Generic On/Off Server model in the BLE Mesh specification
- Receives on/off commands from BLE Mesh clients
- Translates these commands into appropriate IR signals
- Can be provisioned wirelessly into a BLE Mesh network
- Complies with Bluetooth Mesh 1.0 specifications

## Features

- **BLE Mesh Integration**: Fully implements BLE Mesh node functionality with support for provisioning
- **Generic On/Off Server**: Processes standard BLE Mesh On/Off commands
- **IR Signal Transmission**: Sends corresponding IR commands to control external devices
- **Status Reporting**: Provides feedback to the mesh network about current state
- **Arduino ESP-IDF Integration**: Uses Arduino as an ESP-IDF component for simplified development

## Requirements

### Hardware
- ESP32 development board (ESP32, ESP32-S3, or other variants)
- IR LED transmitter connected to the appropriate GPIO
- Power supply via USB or battery
- Optional: IR receiver for testing/learning remote codes

### Software
- ESP-IDF v4.4 or later
- Arduino as an ESP-IDF component
- BLE Mesh libraries (included in ESP-IDF)
- IR remote library

## Setup Instructions

### 1. Configure the Project
Key configuration options:
- Board Selection: Choose your ESP32 development board under Example Configuration
- BLE Mesh Configuration: Default settings should work for most applications

### 2. Build and Flash
Replace PORT with your device's serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux).

## Usage

### BLE Mesh Provisioning
- The device advertises itself as an unprovisioned BLE Mesh device with UUID starting with 0xDD 0xDD
- Use a BLE Mesh provisioner (like the BLE Mesh MQTT client from this project) to provision the device
- After provisioning, the device will be able to receive Generic On/Off commands from the mesh network

### IR Control
When the device receives an On/Off command:
1. It updates its internal state
2. Sends a confirmation back to the mesh network
3. Transmits the corresponding IR signal through the connected IR LED
4. The IR signal controls the target device (TV, AC, etc.)

## Project Structure
- `main.cpp`: Core application code implementing BLE Mesh server functionality and IR control
- `test_remote.cpp`: IR transmission functionality (contains `setupIRTest()` and `send_IR_signal()`)
- `ble_mesh_example_init.h`: BLE Mesh initialization helpers
- `components/arduino/`: Arduino as ESP-IDF component

## Customization
To control different IR devices:
- Modify the `send_IR_signal()` function in `test_remote.cpp` to send the appropriate IR codes for your specific device
- You may need to use an IR receiver to learn codes from your existing remote control

## Troubleshooting
- **Provisioning Issues**: Ensure the device is advertising and the provisioner is configured correctly
- **IR Control Problems**: Check IR LED connections and make sure the IR codes match your device
- **Mesh Communication**: Verify the device is correctly provisioned and app keys are bound

## License
This project is licensed under the Apache License 2.0.

## Acknowledgments
Based on ESP-IDF BLE Mesh examples and Arduino IR libraries.