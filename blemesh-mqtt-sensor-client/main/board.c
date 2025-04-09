/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "iot_button.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "ble_mesh_client.h"
#include "custom_mqtt_client.h"

// Tag used for ESP logging
#define TAG "BOARD"

// GPIO pin number for the button
#define BUTTON_IO_NUM 3
// Active level for the button (0 means active low)
#define BUTTON_ACTIVE_LEVEL 0

// External function declaration for sending BLE mesh sensor messages
extern void example_ble_mesh_send_sensor_message(uint32_t opcode);

/**
 * Array of operation codes that can be sent to sensor nodes
 * Currently only using SENSOR_GET operation to request sensor data
 * Other operations are commented out but could be used for more complex interactions
 */
static uint32_t send_opcode[] = {
    // [0] = ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET,
    // [1] = ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET,
    // [2] = ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET,
    // [3] = ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
    // [4] = ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET,
    [0] = ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
};

// Tracks the number of button presses to cycle through opcodes
static uint8_t press_count;

/**
 * Callback function triggered when button is released
 * Sends a sensor message with the appropriate opcode and increments the press counter
 * 
 * @param arg Pointer to user data (not used in this function)
 */
static void button_tap_cb(void *arg)
{
    example_ble_mesh_send_sensor_message(send_opcode[press_count++]);
    // Wrap around to stay within array bounds
    press_count = press_count % ARRAY_SIZE(send_opcode);
}

/**
 * Initialize the board button
 * Creates a button instance and registers the release callback
 */
static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle)
    {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}

/**
 * Main board initialization function
 * Called during system startup to initialize board-specific features
 */
void board_init(void)
{
    board_button_init();
}
