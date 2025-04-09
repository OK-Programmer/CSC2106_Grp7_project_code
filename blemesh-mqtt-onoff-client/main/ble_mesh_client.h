/**
 * @file ble_mesh_client.h
 * @brief BLE Mesh client implementation for controlling devices in a mesh network
 *
 * This header file defines the interface for a BLE Mesh client that can provision
 * and control devices in a Bluetooth Mesh network. It includes functionality for 
 * sending Generic OnOff SET messages to toggle the state of mesh devices.
 *
 * The implementation uses ESP-IDF's BLE Mesh APIs, including provisioning,
 * networking, configuration, and generic model functionality. It also leverages
 * non-volatile storage for persisting mesh configuration.
 *
 * @note This implementation is part of the CSC2106 Group 7 project for
 * creating a MQTT-BLE Mesh bridge for IoT device control.
 */
#ifndef BLE_MESH_CLIENT_H
#define BLE_MESH_CLIENT_H

// Standard libraries
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// ESP system libraries
#include "esp_log.h"          // ESP logging functionality
#include "nvs_flash.h"        // Non-volatile storage

// BLE Mesh related libraries
#include "esp_ble_mesh_common_api.h"          // Common BLE Mesh APIs
#include "esp_ble_mesh_provisioning_api.h"    // BLE Mesh provisioning
#include "esp_ble_mesh_networking_api.h"      // BLE Mesh networking
#include "esp_ble_mesh_config_model_api.h"    // Configuration Server/Client models
#include "esp_ble_mesh_generic_model_api.h"   // Generic Server/Client models

// Project-specific libraries
#include "ble_mesh_example_init.h"            // BLE Mesh initialization
#include "ble_mesh_example_nvs.h"             // NVS operations for BLE Mesh

/**
 * @brief Initialize and start the BLE Mesh client
 */
void ble_mesh_client_main(void);

/**
 * @brief Send Generic OnOff SET message to control devices in the mesh network
 */
void example_ble_mesh_send_gen_onoff_set(void);

#endif // BLE_MESH_CLIENT_H