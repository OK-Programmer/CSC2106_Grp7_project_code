# CSC2106 Group 7: BLE Mesh MQTT Smart Home Air Conditioner Retrofit

Code for an IoT project to control non-smart air conditioners using BLE Mesh, MQTT, and ESP32 (M5Stamp-C3) boards integrated with Home Assistant. Aims to provide unified control and automation for energy efficiency.

## Project Structure

Contains ESP-IDF code for the following components:

* `blemesh-mqtt-onoff-client`: BLE Mesh client/gateway for AC control commands via MQTT.
* `blemesh-mqtt-onoff-server`: BLE Mesh server node with IR transmitter to control AC.
* `blemesh-mqtt-sensor-client`: BLE Mesh client/gateway for sensor data via MQTT.
* `blemesh-mqtt-sensor-server`: BLE Mesh server node with DHT22 sensor.

## Hardware Requirements

* M5Stamp-C3 (or similar ESP32-C3 boards)
* DHT22 Sensor (for sensor nodes)
* IR Transmitter/Receiver (for control nodes)
* Raspberry Pi 4 (for Home Assistant server)
* Wi-Fi Network

## Software Requirements

* ESP-IDF (v5.1 recommended)
* Arduino Core for ESP32 (as ESP-IDF component)
* MQTT Broker
* Home Assistant OS (on Raspberry Pi)

## Build and Flash

1.  **Setup ESP-IDF:** Follow official Espressif instructions.
2.  **Navigate to Component:** `cd <component_folder>` (e.g., `cd blemesh-mqtt-sensor-server`)
3.  **Configure:** `idf.py menuconfig` (Set Wi-Fi, MQTT, GPIO pins, etc.)
4.  **Build:** `idf.py build`
5.  **Flash:** `idf.py -p PORT flash`
6.  **Monitor:** `idf.py -p PORT monitor`

(Replace `PORT` with your device's serial port)

## Authors (Group 7)

* James Patrick Francisco Gonzales
* Jiang Weimin
* Chew Liang Zhi
* Leo Oh Kang Weng
* Joween Ang

## License

MIT License