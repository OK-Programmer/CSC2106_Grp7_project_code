#ifndef BLE_MESH_CLIENT_H
#define BLE_MESH_CLIENT_H

/* Standard libraries */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* ESP system libraries */
#include "esp_log.h"          // ESP logging functionality
#include "nvs_flash.h"        // Non-volatile storage

/* BLE Mesh core APIs */
#include "esp_ble_mesh_common_api.h"       // Common BLE Mesh APIs
#include "esp_ble_mesh_provisioning_api.h"  // Provisioning functionality
#include "esp_ble_mesh_networking_api.h"    // Networking operations
#include "esp_ble_mesh_config_model_api.h"  // Configuration model
#include "esp_ble_mesh_generic_model_api.h" // Generic model operations

/* Project specific headers */
#include "ble_mesh_example_init.h"          // BLE Mesh initialization
#include "ble_mesh_example_nvs.h"           // Non-volatile storage for BLE Mesh

/**
 * @brief Initialize and start the BLE Mesh client functionality
 */
void ble_mesh_client_main(void);

/**
 * @brief Send a sensor request message
 * @param opcode Operation code defining the type of message
 */
void example_ble_mesh_send_sensor_message(uint32_t opcode);

/**
 * @brief Check if the device has been provisioned to a network
 * @return true if provisioned, false otherwise
 */
bool get_is_provisioned(void);

#endif // BLE_MESH_CLIENT_H