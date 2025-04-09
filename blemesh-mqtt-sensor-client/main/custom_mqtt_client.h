/**
 * @file custom_mqtt_client.h
 * @brief Header file for custom MQTT client implementation
 *
 * Provides interface for MQTT client functionality to connect to a broker
 * and publish sensor data over MQTT protocol.
 */

#ifndef CUSTOM_MQTT_CLIENT_H
#define CUSTOM_MQTT_CLIENT_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"         /* ESP32 system functions */
#include "nvs_flash.h"          /* Non-volatile storage */
#include "esp_event.h"          /* Event handling */
#include "esp_netif.h"          /* Network interface */
#include "protocol_examples_common.h" /* Common example functionality */
#include "esp_log.h"            /* Logging functions */
#include "mqtt_client.h"        /* ESP MQTT client library */

/**
 * @brief Initialize and start the MQTT client
 * 
 * Sets up the MQTT client, connects to the broker and starts the client task.
 */
extern void mqtt_client_main(void);

/**
 * @brief Publishes data to a specified MQTT topic
 * 
 * @param topic MQTT topic to publish to
 * @param data Data payload to publish
 * @param len Length of the data payload
 */
extern void publish_data(const char *topic, const char *data, int len);

#endif // CUSTOM_MQTT_CLIENT_H