/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* 
 * This is a BLE Mesh sensor server example that reads temperature and humidity data
 * from a DHT sensor and makes it available over BLE Mesh network.
 */

#include <Arduino.h> // Required for Arduino API compatibility
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <DHT.h>  // DHT temperature and humidity sensor library

// ESP32 BLE Mesh related includes
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "ble_mesh_example_init.h"
#include "board.h"

// External functions defined in another file for DHT sensor handling
extern void tempsetup();  // Initialize the DHT sensor
extern void temploop(float *humidity, float *temperature, float *fahrenheit, float *heatIndexC, float *heatIndexF);  // Read sensor values

#define TAG "EXAMPLE"  // Log tag for debugging

#define CID_ESP     0x02E5  // Company ID for ESP (Espressif)

/* Sensor Property ID definitions according to Bluetooth Mesh Model specification */
#define SENSOR_PROPERTY_ID_0        0x0056  /* Present Indoor Ambient Temperature */
#define SENSOR_PROPERTY_ID_1        0x005B  /* Present Outdoor Ambient Temperature */

/* 
 * Global variables to store sensor readings
 * The characteristic of these properties is "Temperature 8", which is
 * used to represent temperature with a unit of 0.5 degree Celsius.
 * Minimum value: -64.0, maximum value: 63.5.
 * A value of 0xFF represents 'value is not known'.
 */
static int8_t g_humidity = 40;     /* Initial value for humidity */
static int8_t g_temperature = 60;  /* Initial value for temperature (30°C) */

/* Sensor characteristic settings as defined in the Mesh Model specification */
#define SENSOR_POSITIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION      ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD       ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL      ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL

/* Unique device identifier for BLE Mesh provisioning */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

/* Configuration server model - handles network configuration */
static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,              // Enable message relay functionality
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20), // Relay retransmission settings
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,            // Enable mesh beacon
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,    // Enable GATT Proxy if configured
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,      // Enable Friend feature if configured
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,                                 // Default Time To Live for messages
};

/* Define buffers for sensor data storage */
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_0, 1);  // Buffer for humidity data
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_1, 1);  // Buffer for temperature data

/* 
 * Define sensor states for both sensors (humidity and temperature)
 * Each state includes property ID, descriptor settings, and data format
 */
static esp_ble_mesh_sensor_state_t sensor_states[2] = {
    /* 
     * Multiple instances of Sensor states may be present within the same model,
     * provided each has a unique Sensor Property ID to allow differentiation.
     * These are known as multisensors.
     */
    [0] = {  // Humidity sensor state
        .sensor_property_id = SENSOR_PROPERTY_ID_0,
        /* 
         * Sensor Descriptor state represents attributes describing sensor data.
         * This state doesn't change throughout the lifetime of an element.
         */
        .descriptor = {
            .positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function = SENSOR_SAMPLE_FUNCTION,
            .measure_period = SENSOR_MEASURE_PERIOD,
            .update_interval = SENSOR_UPDATE_INTERVAL,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 0, /* 0 indicates length is 1 byte */
            .raw_value = &sensor_data_0,  // Points to the buffer holding humidity data
        },
    },
    [1] = {  // Temperature sensor state
        .sensor_property_id = SENSOR_PROPERTY_ID_1,
        .descriptor = {
            .positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function = SENSOR_SAMPLE_FUNCTION,
            .measure_period = SENSOR_MEASURE_PERIOD,
            .update_interval = SENSOR_UPDATE_INTERVAL,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 0, /* 0 indicates length is 1 byte */
            .raw_value = &sensor_data_1,  // Points to the buffer holding temperature data
        },
    },
};

/* Define publication settings for sensor model - 20 octets can hold two Sensor Descriptor state values */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 20, ROLE_NODE);

/* Sensor Server model - handles sensor data reporting */
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,  // Application handles GET responses
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,  // Application handles SET responses
    },
    .state_count = ARRAY_SIZE(sensor_states),  // Number of sensor states
    .states = sensor_states,                   // Pointer to sensor states array
};

/* Define publication settings for sensor setup model */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 20, ROLE_NODE);

/* Sensor Setup Server model - handles sensor configuration */
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,  // Application handles GET responses
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,  // Application handles SET responses
    },
    .state_count = ARRAY_SIZE(sensor_states),  // Number of sensor states
    .states = sensor_states,                   // Pointer to sensor states array
};

/* Define root element models - Configuration Server, Sensor Server, and Sensor Setup Server */
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),
};

/* Define elements in the device - just one root element in this example */
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

/* Define the device composition data */
static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,                       // Company ID
    .element_count = ARRAY_SIZE(elements), // Number of elements
    .elements = elements,                  // Pointer to elements array
};

/* Define provisioning settings */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,  // Device UUID for provisioning
};

/**
 * @brief Callback triggered when provisioning is complete
 * @param net_idx Network index
 * @param addr Node address
 * @param flags Flags
 * @param iv_index IV Index
 */
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
    board_led_operation(LED_G, LED_OFF);  // Turn off green LED after provisioning

    /* Initialize the sensor data with default values */
    net_buf_simple_add_u8(&sensor_data_0, g_humidity);     // Add humidity to buffer
    net_buf_simple_add_u8(&sensor_data_1, g_temperature);  // Add temperature to buffer
}

/**
 * @brief Callback for handling provisioning events
 * @param event BLE Mesh provisioning event
 * @param param Event parameters
 */
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

/**
 * @brief Callback for handling configuration server events
 * @param event BLE Mesh config server event
 * @param param Event parameters
 */
static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

/* Sensor descriptor structure conforming to the BLE Mesh specification */
struct example_sensor_descriptor {
    uint16_t sensor_prop_id;
    uint32_t pos_tolerance:12,
             neg_tolerance:12,
             sample_func:8;
    uint8_t  measure_period;
    uint8_t  update_interval;
} __attribute__((packed));

/**
 * @brief Send sensor descriptor status as response to descriptor get message
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_descriptor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct example_sensor_descriptor descriptor = {0};
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    // Allocate memory for descriptor status
    status = (uint8_t *)calloc(1, ARRAY_SIZE(sensor_states) * ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
    if (!status) {
        ESP_LOGE(TAG, "No memory for sensor descriptor status!");
        return;
    }

    if (param->value.get.sensor_descriptor.op_en == false) {
        /* 
         * If Property ID is omitted in the get message, respond with 
         * descriptors for all sensors in this server
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status + length, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length += ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
        }
        goto send;
    }

    // Look for the requested property ID
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        if (param->value.get.sensor_descriptor.property_id == sensor_states[i].sensor_property_id) {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length = ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
            goto send;
        }
    }

    /* 
     * If requested property ID doesn't exist, just include the property ID 
     * and omit other fields
     */
    memcpy(status, &param->value.get.sensor_descriptor.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Descriptor", status, length);

    // Send the response message
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Descriptor Status");
    }
    free(status);
}

/**
 * @brief Send sensor cadence status (not fully supported in this example)
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_cadence_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    // Send only property ID since sensor cadence state is not supported
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_cadence.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Cadence Status");
    }
}

/**
 * @brief Send sensor settings status (not fully supported in this example)
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_settings_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    // Send only property ID since sensor settings state is not supported
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_settings.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Settings Status");
    }
}

/* Sensor setting structure for responses */
struct example_sensor_setting {
    uint16_t sensor_prop_id;
    uint16_t sensor_setting_prop_id;
} __attribute__((packed));

/**
 * @brief Send sensor setting status (not fully supported in this example)
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_setting_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct example_sensor_setting setting = {0};
    esp_err_t err;

    /* 
     * Since sensor settings are not implemented, return just the property IDs
     * without access or raw value fields
     */
    setting.sensor_prop_id = param->value.get.sensor_setting.property_id;
    setting.sensor_setting_prop_id = param->value.get.sensor_setting.setting_property_id;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_STATUS,
            sizeof(setting), (uint8_t *)&setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Setting Status");
    }
}

/**
 * @brief Extract sensor data from sensor state and format according to BLE Mesh spec
 * @param state Pointer to sensor state
 * @param data Output buffer for formatted data
 * @return Number of bytes written to data buffer
 */
static uint16_t example_ble_mesh_get_sensor_data(esp_ble_mesh_sensor_state_t *state, uint8_t *data)
{
    uint8_t mpid_len = 0, data_len = 0;
    uint32_t mpid = 0;

    if (state == NULL || data == NULL) {
        ESP_LOGE(TAG, "%s, Invalid parameter", __func__);
        return 0;
    }

    if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
        /* For zero-length sensor data, the length is 0x7F, and the format is Format B */
        mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
        mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        data_len = 0;
    } else {
        if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN;
        } else {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        /* Use "state->sensor_data.length + 1" because the length is zero-based */
        data_len = state->sensor_data.length + 1;
    }

    memcpy(data, &mpid, mpid_len);
    memcpy(data + mpid_len, state->sensor_data.raw_value->data, data_len);

    return (mpid_len + data_len);
}

/**
 * @brief Send sensor data status response to a get request
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    uint32_t mpid = 0;
    esp_err_t err;
    int i;

    /* Calculate buffer size needed for all sensor data */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        esp_ble_mesh_sensor_state_t *state = &sensor_states[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        } else {
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            } else {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = (uint8_t *)calloc(1, buf_size);
    if (!status) {
        ESP_LOGE(TAG, "No memory for sensor status!");
        return;
    }

    if (param->value.get.sensor_data.op_en == false) {
        /* If no property ID specified, include data for all sensors */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
            length += example_ble_mesh_get_sensor_data(&sensor_states[i], status + length);
        }
        goto send;
    }

    /* If specific property ID requested, find and include only that one */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        if (param->value.get.sensor_data.property_id == sensor_states[i].sensor_property_id) {
            length = example_ble_mesh_get_sensor_data(&sensor_states[i], status);
            goto send;
        }
    }

    /* If requested property not found, just include the property ID with zero length */
    mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN,
            param->value.get.sensor_data.property_id);
    memcpy(status, &mpid, ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
    length = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);

    /* Send the sensor data status message */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Status");
    }
    free(status);
}

/**
 * @brief Send sensor column status (not fully implemented in this example)
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_column_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;

    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN + param->value.get.sensor_column.raw_value_x->len;

    status = (uint8_t *)calloc(1, length);
    if (!status) {
        ESP_LOGE(TAG, "No memory for sensor column status!");
        return;
    }

    memcpy(status, &param->value.get.sensor_column.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    memcpy(status + ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN, param->value.get.sensor_column.raw_value_x->data,
        param->value.get.sensor_column.raw_value_x->len);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
    free(status);
}

/**
 * @brief Send sensor series status (not fully implemented in this example)
 * @param param Sensor server callback parameters
 */
static void example_ble_mesh_send_sensor_series_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_series.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
}

/**
 * @brief Callback for handling sensor server events
 * @param event Sensor server event type
 * @param param Event parameters
 */
static void example_ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                              esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
        event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    switch (event) {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        /* Handle different GET requests */
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET");
            example_ble_mesh_send_sensor_descriptor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET");
            example_ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            example_ble_mesh_send_sensor_settings_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            example_ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            example_ble_mesh_send_sensor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET");
            example_ble_mesh_send_sensor_column_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET");
            example_ble_mesh_send_sensor_series_status(param);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04" PRIx32, param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        /* Handle different SET requests */
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET");
            example_ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET");
            example_ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK");
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04" PRIx32, param->ctx.recv_op);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Server event %d", event);
        break;
    }
}

/**
 * @brief Initialize the BLE Mesh functionality
 * @return ESP_OK on success, otherwise an error code
 */
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    // Register callback functions
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(example_ble_mesh_sensor_server_cb);

    // Initialize BLE Mesh stack
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    // Enable provisioning
    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    // Turn on green LED to indicate device is ready for provisioning
    board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh sensor server initialized");

    return ESP_OK;
}

/**
 * @brief Application main entry point
 */
extern "C" void app_main(void)
{
    esp_err_t err;
    
    ESP_LOGI(TAG, "Initializing...");
    
    // Initialize NVS flash storage
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // Initialize board-specific hardware
    board_init();
    
    // Initialize Bluetooth controller and stack
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }
    
    // Get device UUID for provisioning
    ble_mesh_get_dev_uuid(dev_uuid);
    
    // Initialize the BLE Mesh Subsystem
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    
    // -----------------------------
    // ARDUINO + Temperature Sensor Initialization
    // -----------------------------
    
    // 1. Initialize Arduino environment for using Arduino APIs
    initArduino();

    // 2. Initialize the DHT temperature sensor
    tempsetup();

    // 3. Main application loop - repeatedly read sensor data and update BLE Mesh values
    while (true) {
        // Variables to store sensor readings
        static float humidity = 0.0;
        static float temperature = 0.0;
        static float fahrenheit = 0.0;
        static float heatIndexC = 0.0;
        static float heatIndexF = 0.0;
        
        // Read values from DHT sensor
        temploop(&humidity, &temperature, &fahrenheit, &heatIndexC, &heatIndexF);
        
        // Update global variables with new readings (converting to int8_t)
        g_humidity = static_cast<int8_t>(humidity);  
        g_temperature = static_cast<int8_t>(temperature); 
        
        // Log current values
        ESP_LOGI(TAG, "humidity: %d, temp: %d", g_humidity, g_temperature);
        
        // Brief delay to avoid too frequent readings
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
