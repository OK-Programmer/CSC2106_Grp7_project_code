/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file custom_mqtt_client.c
 * @brief Implementation of a custom MQTT client using MQTT v5.0 protocol
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "custom_mqtt_client.h"

/* Global MQTT client handle */
static esp_mqtt_client_handle_t mqtt_client;

static const char *TAG = "mqtt5_example";

/* External function declaration for Bluetooth Mesh functionality */
extern void example_ble_mesh_send_gen_onoff_set(void);

/**
 * @brief Logs an error message if the error code is non-zero
 * 
 * @param message Error message prefix
 * @param error_code The error code to check
 */
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/* Define user properties to include in MQTT messages */
static esp_mqtt5_user_property_item_t user_property_arr[] = {
    {"board", "esp32"},
    {"u", "user"},
    {"p", "password"}};

#define USE_PROPERTY_ARR_SIZE sizeof(user_property_arr) / sizeof(esp_mqtt5_user_property_item_t)

/* Configuration for MQTT5 publish properties */
static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,                // Indicates the format of the payload (1 = UTF-8)
    .message_expiry_interval = 1000,              // Message expires after this many seconds
    .topic_alias = 0,                             // No topic alias used
    .response_topic = "/topic/test/response",     // Topic to send responses to
    .correlation_data = "123456",                 // Data for correlating request with response
    .correlation_data_len = 6,
};

/* Configuration for the first subscription properties */
static esp_mqtt5_subscribe_property_config_t subscribe_property = {
    .subscribe_id = 25555,                        // Subscription identifier
    .no_local_flag = false,                       // Receive own published messages
    .retain_as_published_flag = false,            // Do not keep retain flag
    .retain_handle = 0,                           // Retain handling option
    .is_share_subscribe = true,                   // Enable shared subscription
    .share_name = "group1",                       // Shared subscription group name
};

/* Configuration for an alternative subscription properties */
static esp_mqtt5_subscribe_property_config_t subscribe1_property = {
    .subscribe_id = 25555,
    .no_local_flag = true,                        // Don't receive own published messages
    .retain_as_published_flag = false,
    .retain_handle = 0,
};

/* Configuration for unsubscribe properties */
static esp_mqtt5_unsubscribe_property_config_t unsubscribe_property = {
    .is_share_subscribe = true,
    .share_name = "group1",
};

/* Configuration for disconnect properties */
static esp_mqtt5_disconnect_property_config_t disconnect_property = {
    .session_expiry_interval = 60,                // Session expires after 60 seconds
    .disconnect_reason = 0,                       // Normal disconnect reason
};

/**
 * @brief Prints all user properties from an MQTT5 message
 * 
 * @param user_property Handle to the user properties
 */
static void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property)
    {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count)
        {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK)
            {
                for (int i = 0; i < count; i++)
                {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

/**
 * @brief Event handler registered to receive MQTT events
 *
 * This function is called by the MQTT client event loop for various events.
 *
 * @param handler_args User data registered to the event
 * @param base Event base for the handler (always MQTT Base in this example)
 * @param event_id The id for the received event
 * @param event_data The data for the event, esp_mqtt_event_handle_t
 */
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Set up subscription with user properties
        esp_mqtt5_client_set_user_property(&subscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_subscribe_property(client, &subscribe_property);
        // msg_id = esp_mqtt_client_subscribe(client, "/CSC2106/state", 0);
        esp_mqtt5_client_delete_user_property(subscribe_property.user_property);
        subscribe_property.user_property = NULL;
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        // Publish a message after successful subscription
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/CSC2106/state", "toggle", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        // Set disconnect properties and disconnect client
        esp_mqtt5_client_set_user_property(&disconnect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_disconnect_property(client, &disconnect_property);
        esp_mqtt5_client_delete_user_property(disconnect_property.user_property);
        disconnect_property.user_property = NULL;
        esp_mqtt_client_disconnect(client);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        // Log received message properties
        ESP_LOGI(TAG, "payload_format_indicator is %d", event->property->payload_format_indicator);
        ESP_LOGI(TAG, "response_topic is %.*s", event->property->response_topic_len, event->property->response_topic);
        ESP_LOGI(TAG, "correlation_data is %.*s", event->property->correlation_data_len, event->property->correlation_data);
        ESP_LOGI(TAG, "content_type is %.*s", event->property->content_type_len, event->property->content_type);
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        // Handle different types of errors
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/**
 * @brief Initialize and start the MQTT5 client
 */
static void mqtt5_app_start(void)
{
    // Configure MQTT5 connection properties
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,            // Session expires after 10 seconds
        .maximum_packet_size = 1024,              // Maximum size of MQTT packets
        .receive_maximum = 65535,                 // Maximum number of inflight messages
        .topic_alias_maximum = 2,                 // Maximum number of topic aliases
        .request_resp_info = true,                // Request response information
        .request_problem_info = true,             // Request problem information
        .will_delay_interval = 10,                // Delay will message by 10 seconds after disconnect
        .payload_format_indicator = true,         // Payload is in UTF-8 format
        .message_expiry_interval = 10,            // Message expires after 10 seconds
        .response_topic = "/test/response",       // Topic for responses
        .correlation_data = "123456",             // Correlation data
        .correlation_data_len = 6,
    };

    // Configure MQTT client
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,  // MQTT broker URL from config
        .session.protocol_ver = MQTT_PROTOCOL_V_5, // Use MQTT v5 protocol
        .network.disable_auto_reconnect = true,   // Disable automatic reconnection
        .credentials.username = "123",            // Username for authentication
        .credentials.authentication.password = "456", // Password for authentication
        .session.last_will.topic = "/topic/will", // LWT topic
        .session.last_will.msg = "i will leave",  // LWT message
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,               // QoS 1 for LWT
        .session.last_will.retain = true,         // Retain LWT message
    };

    // Initialize the MQTT client
    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(mqtt_client, &connect_property);

    /* If you call esp_mqtt5_client_set_user_property to set user properties, DO NOT forget to delete them.
     * esp_mqtt5_client_set_connect_property will malloc buffer to store the user_property and you can delete it after
     */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* Register event handler and start the client */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

/**
 * @brief Publish data to an MQTT topic
 * 
 * @param topic The MQTT topic to publish to
 * @param data The data payload to publish
 * @param len Length of the data payload
 */
void publish_data(const char *topic, const char *data, int len)
{
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, len, 1, 0);
    if (msg_id != -1)
    {
        ESP_LOGI(TAG, "Data published successfully, msg_id=%d", msg_id);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to publish data");
    }
}

/**
 * @brief Main entry point for the MQTT client
 * 
 * Initializes the system and starts the MQTT client
 */
void mqtt_client_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // Set log levels for different components
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    // Initialize NVS, network interface, and event loop
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // Start the MQTT client
    mqtt5_app_start();
}
