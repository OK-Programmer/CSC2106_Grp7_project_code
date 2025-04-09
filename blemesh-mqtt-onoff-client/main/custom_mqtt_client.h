/**
 * @file custom_mqtt_client.h
 * @brief Header file for custom MQTT client implementation
 */
#ifndef CUSTOM_MQTT_CLIENT_H
#define CUSTOM_MQTT_CLIENT_H

/* Standard library includes */
#include <stdio.h>      /* Standard I/O functions */
#include <stdint.h>     /* Standard integer types */
#include <stddef.h>     /* Standard definitions */
#include <string.h>     /* String handling functions */

/* ESP-IDF includes */
#include "esp_system.h"  /* ESP system functions */
#include "nvs_flash.h"   /* Non-volatile storage flash API */
#include "esp_event.h"   /* ESP event handling */
#include "esp_netif.h"   /* ESP network interface */
#include "protocol_examples_common.h"  /* Common functions for protocol examples */
#include "esp_log.h"     /* ESP logging functions */
#include "mqtt_client.h"  /* ESP MQTT client */

/**
 * @brief Entry point function for MQTT client
 * 
 * Initializes and starts the MQTT client functionality
 */
extern void mqtt_client_main(void);

#endif // CUSTOM_MQTT_CLIENT_H