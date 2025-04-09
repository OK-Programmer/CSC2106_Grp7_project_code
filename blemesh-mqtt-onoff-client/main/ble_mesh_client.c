/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE Mesh Client Application
 * 
 * This file implements a BLE Mesh provisioner that can discover, provision and
 * configure BLE Mesh nodes, and control their Generic OnOff Server model.
 * 
 * The client communicates with server nodes to toggle their state and
 * measures round-trip latency of the mesh network communications.
 */

/* Include standard C library headers */
#include <stdio.h>     /* Standard I/O functions */
#include <string.h>    /* String manipulation functions */
#include <inttypes.h>  /* Integer format conversion macros */

/* Include ESP-IDF system headers */
#include "esp_log.h"   /* ESP logging utilities */
#include "nvs_flash.h" /* Non-volatile storage (NVS) functions */

/* BLE Mesh related header files */
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"
#include "esp_timer.h"

#include "ble_mesh_client.h"

#define TAG "EXAMPLE"

/* Company ID for ESP BLE Mesh */
#define CID_ESP 0x02E5

/* Provisioner unicast address */
#define PROV_OWN_ADDR 0x0001

/* Time To Live for mesh messages */
#define MSG_SEND_TTL 3
/* Message timeout value */
#define MSG_TIMEOUT 0
/* Message role */
#define MSG_ROLE ROLE_PROVISIONER

/* Page number for Composition Data */
#define COMP_DATA_PAGE_0 0x00

/* Application key index */
#define APP_KEY_IDX 0x0000
/* Application key octet value */
#define APP_KEY_OCTET 0x12

/* Macros to extract 1 or 2 octets from composition data */
#define COMP_DATA_1_OCTET(msg, offset) (msg[offset])
#define COMP_DATA_2_OCTET(msg, offset) (msg[offset + 1] << 8 | msg[offset])

/* Device UUID used for provisioning */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN];
/* Address of the server node */
static uint16_t server_address = ESP_BLE_MESH_ADDR_UNASSIGNED;
/* On/off state to be sent in Generic OnOff Set message */
static uint8_t onoff;
/* Timestamp for measuring round-trip time */
static int64_t send_time;

/* Structure for storing network key, app key and app key index */
static struct esp_ble_mesh_key
{
    uint16_t net_idx;  /* Network key index */
    uint16_t app_idx;  /* Application key index */
    uint8_t app_key[ESP_BLE_MESH_OCTET16_LEN];  /* Application key */
} prov_key;

/* Client structure for Generic OnOff model */
static esp_ble_mesh_client_t onoff_client;

/* Client structure for Configuration model */
static esp_ble_mesh_client_t config_client;

/* Define publication context for Generic OnOff Client model */
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 1, ROLE_PROVISIONER);

/* Define root element models */
static esp_ble_mesh_model_t root_models[] = {
    // ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),  /* Configuration Server model is commented out */
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),    /* Configuration Client model */
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),  /* Generic OnOff Client model */
};

/* Define elements - only root element in this example */
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),  /* Element with root models, no vendor models */
};

/* Define BLE Mesh composition data */
static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,                      /* Company ID */
    .element_count = ARRAY_SIZE(elements), /* Number of elements */
    .elements = elements,                /* Elements array */
};

/* Define provisioning parameters */
static esp_ble_mesh_prov_t provision = {
    .prov_uuid = dev_uuid,               /* Device UUID */
    .prov_unicast_addr = PROV_OWN_ADDR,  /* Provisioner address */
    .prov_start_address = 0x0005,        /* Start address for provisioning */
};

/**
 * @brief Set common parameters for client messages
 * 
 * @param common Common message parameters structure to be filled
 * @param node Target node information
 * @param model Client model that is sending the message
 * @param opcode Operation code for the message
 */
static void example_ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                            esp_ble_mesh_node_t *node,
                                            esp_ble_mesh_model_t *model, uint32_t opcode)
{
    common->opcode = opcode;             /* Message opcode */
    common->model = model;               /* Client model sending the message */
    common->ctx.net_idx = prov_key.net_idx; /* Network index */
    common->ctx.app_idx = prov_key.app_idx; /* Application key index */
    common->ctx.addr = node->unicast_addr;  /* Target node address */
    common->ctx.send_ttl = MSG_SEND_TTL;    /* Time To Live */
    common->msg_timeout = MSG_TIMEOUT;      /* Message timeout */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common->msg_role = MSG_ROLE;         /* Message role (for backward compatibility) */
#endif
}

/* Callback function to handle provisioning completion */
static esp_err_t prov_complete(uint16_t node_index, const esp_ble_mesh_octet16_t uuid,
                               uint16_t primary_addr, uint8_t element_num, uint16_t net_idx)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get = {0};
    esp_ble_mesh_node_t *node = NULL;
    char name[11] = {'\0'};
    esp_err_t err = ESP_OK;

    /* Log provisioning completion details */
    ESP_LOGI(TAG, "node_index %u, primary_addr 0x%04x, element_num %u, net_idx 0x%03x",
             node_index, primary_addr, element_num, net_idx);
    ESP_LOG_BUFFER_HEX("uuid", uuid, ESP_BLE_MESH_OCTET16_LEN);

    /* Store the server address */
    server_address = primary_addr;

    /* Set a name for the provisioned node */
    sprintf(name, "%s%02x", "NODE-", node_index);
    err = esp_ble_mesh_provisioner_set_node_name(node_index, name);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set node name");
        return ESP_FAIL;
    }

    /* Retrieve the node information using its primary address */
    node = esp_ble_mesh_provisioner_get_node_with_addr(primary_addr);
    if (node == NULL)
    {
        ESP_LOGE(TAG, "Failed to get node 0x%04x info", primary_addr);
        return ESP_FAIL;
    }

    /* Send a Composition Data Get message to the node */
    example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Composition Data Get");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Callback function to handle unprovisioned device advertisement packets */
static void recv_unprov_adv_pkt(uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    esp_err_t err = ESP_OK;

    /* Log details of the unprovisioned device */
    ESP_LOG_BUFFER_HEX("Device address", addr, BD_ADDR_LEN);
    ESP_LOGI(TAG, "Address type 0x%02x, adv type 0x%02x", addr_type, adv_type);
    ESP_LOG_BUFFER_HEX("Device UUID", dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    ESP_LOGI(TAG, "oob info 0x%04x, bearer %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    /* Prepare the device information for provisioning */
    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (esp_ble_mesh_addr_type_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (esp_ble_mesh_prov_bearer_t)bearer;

    /* Add the unprovisioned device and start provisioning */
    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
                                                  ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start provisioning device");
    }
}

/* Callback function to handle BLE Mesh provisioning events */
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                            esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        /* Log provisioning registration completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        /* Log provisioning enable completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        /* Log provisioning disable completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        /* Handle unprovisioned device advertisement packet */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        /* Log provisioning link open event */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT, bearer %s",
                 param->provisioner_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        /* Log provisioning link close event */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT, bearer %s, reason 0x%02x",
                 param->provisioner_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        /* Handle provisioning completion */
        prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                      param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                      param->provisioner_prov_complete.netkey_idx);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        /* Log unprovisioned device addition completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        /* Log device UUID match setting completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code %d", param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
        /* Log node name setting completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d", param->provisioner_set_node_name_comp.err_code);
        if (param->provisioner_set_node_name_comp.err_code == 0)
        {
            const char *name = esp_ble_mesh_provisioner_get_node_name(param->provisioner_set_node_name_comp.node_index);
            if (name)
            {
                ESP_LOGI(TAG, "Node %d name %s", param->provisioner_set_node_name_comp.node_index, name);
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        /* Log local AppKey addition completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == 0)
        {
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            esp_err_t err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                                                                                 ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to bind AppKey to onoff client");
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        /* Log AppKey binding to model completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT:
        /* Log node composition data storage completion */
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT, err_code %d", param->provisioner_store_node_comp_data_comp.err_code);
        break;
    default:
        break;
    }
}


/* Function to parse and log the composition data of a node */
static void example_ble_mesh_parse_node_comp_data(const uint8_t *data, uint16_t length)
{
    uint16_t cid, pid, vid, crpl, feat;
    uint16_t loc, model_id, company_id;
    uint8_t nums, numv;
    uint16_t offset;
    int i;

    /* Extract and log the composition data header */
    cid = COMP_DATA_2_OCTET(data, 0);
    pid = COMP_DATA_2_OCTET(data, 2);
    vid = COMP_DATA_2_OCTET(data, 4);
    crpl = COMP_DATA_2_OCTET(data, 6);
    feat = COMP_DATA_2_OCTET(data, 8);
    offset = 10;

    ESP_LOGI(TAG, "********************** Composition Data Start **********************");
    ESP_LOGI(TAG, "* CID 0x%04x, PID 0x%04x, VID 0x%04x, CRPL 0x%04x, Features 0x%04x *", cid, pid, vid, crpl, feat);

    /* Parse and log each element's models */
    for (; offset < length;)
    {
        loc = COMP_DATA_2_OCTET(data, offset);
        nums = COMP_DATA_1_OCTET(data, offset + 2);
        numv = COMP_DATA_1_OCTET(data, offset + 3);
        offset += 4;
        ESP_LOGI(TAG, "* Loc 0x%04x, NumS 0x%02x, NumV 0x%02x *", loc, nums, numv);

        /* Log SIG models */
        for (i = 0; i < nums; i++)
        {
            model_id = COMP_DATA_2_OCTET(data, offset);
            ESP_LOGI(TAG, "* SIG Model ID 0x%04x *", model_id);
            offset += 2;
        }

        /* Log vendor models */
        for (i = 0; i < numv; i++)
        {
            company_id = COMP_DATA_2_OCTET(data, offset);
            model_id = COMP_DATA_2_OCTET(data, offset + 2);
            ESP_LOGI(TAG, "* Vendor Model ID 0x%04x, Company ID 0x%04x *", model_id, company_id);
            offset += 4;
        }
    }
    ESP_LOGI(TAG, "*********************** Composition Data End ***********************");
}

/* Callback function to handle BLE Mesh configuration client events */
static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                              esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    static uint16_t wait_model_id, wait_cid;
    esp_ble_mesh_node_t *node = NULL;
    esp_err_t err = ESP_OK;

    /* Log the configuration client event details */
    ESP_LOGI(TAG, "Config client, event %u, addr 0x%04x, opcode 0x%04" PRIx32,
             event, param->params->ctx.addr, param->params->opcode);

    /* Handle errors in configuration client messages */
    if (param->error_code)
    {
        ESP_LOGE(TAG, "Send config client message failed (err %d)", param->error_code);
        return;
    }

    /* Retrieve the node information using its address */
    node = esp_ble_mesh_provisioner_get_node_with_addr(param->params->ctx.addr);
    if (!node)
    {
        ESP_LOGE(TAG, "Node 0x%04x not exists", param->params->ctx.addr);
        return;
    }

    switch (event)
    {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        /* Handle Composition Data Get response */
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET)
        {
            ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
                               param->status_cb.comp_data_status.composition_data->len);
            example_ble_mesh_parse_node_comp_data(param->status_cb.comp_data_status.composition_data->data,
                                                  param->status_cb.comp_data_status.composition_data->len);

            /* Store the node's composition data */
            err = esp_ble_mesh_provisioner_store_node_comp_data(param->params->ctx.addr,
                                                                param->status_cb.comp_data_status.composition_data->data,
                                                                param->status_cb.comp_data_status.composition_data->len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to store node composition data");
                break;
            }

            /* Send AppKey Add message */
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to send Config AppKey Add");
            }
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        /* Handle AppKey Add response */
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD)
        {
            /* Send Model App Bind message */
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
            set.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to send Config Model App Bind");
                return;
            }
            wait_model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
            wait_cid = ESP_BLE_MESH_CID_NVAL;
        }
        /* Handle Model App Bind response */
        else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND)
        {
            ESP_LOGW(TAG, "Provision and config successfully");
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        /* Handle Composition Data Status publish event */
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS)
        {
            ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
                               param->status_cb.comp_data_status.composition_data->len);
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        /* Handle timeout events for various configuration messages */
        switch (param->params->opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET:
        {
            /* Resend Composition Data Get message */
            esp_ble_mesh_cfg_client_get_state_t get = {0};
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to send Config Composition Data Get");
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            /* Resend AppKey Add message */
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to send Config AppKey Add");
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            /* Resend Model App Bind message */
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = wait_model_id;
            set.model_app_bind.company_id = wait_cid;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to send Config Model App Bind");
            }
            break;
        default:
            break;
        }
        break;

    default:
        ESP_LOGE(TAG, "Invalid config client event %u", event);
        break;
    }
}
/**
 * @brief Send a Generic OnOff Set message to toggle the on/off state
 */
void example_ble_mesh_send_gen_onoff_set(void)
{
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    /* Set common parameters for the Generic OnOff Set message */
    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET;
    common.model = onoff_client.model;
    common.ctx.net_idx = prov_key.net_idx;
    common.ctx.app_idx = prov_key.app_idx;
    common.ctx.addr = server_address;   /* Target server address */
    common.ctx.send_ttl = 3;            /* Time To Live */
    common.msg_timeout = 0;             /* Use default timeout from menuconfig */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common.msg_role = ROLE_NODE;        /* Message role for backward compatibility */
#endif

    /* Set the on/off state to be sent */
    set.onoff_set.op_en = false;        /* Disable optional parameters */
    set.onoff_set.onoff = onoff;        /* Current on/off state */

    /* Send the Generic OnOff Set message */
    err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err) {
        ESP_LOGE(TAG, "Send Generic OnOff Set failed");
        return;
    } else {
        send_time = esp_timer_get_time(); /* Record the send timestamp */
        ESP_LOGI(TAG, "Sent OnOff Set at: %lld us", send_time);
        ESP_LOGI(TAG, "Send Generic OnOff Set: %s", onoff ? "ON" : "OFF");
    }

    /* Toggle the on/off state for the next message */
    onoff = !onoff;
}

/**
 * @brief Callback function for handling Generic Client events
 */
static void example_ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                            esp_ble_mesh_generic_client_cb_param_t *param)
{
    ESP_LOGI(TAG, "Generic client, event %u, error code %d, opcode is 0x%04" PRIx32,
        event, param->error_code, param->params->opcode);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
        /* Handle Generic OnOff Get response */
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT");
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, onoff %d", param->status_cb.onoff_status.present_onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
        /* Handle Generic OnOff Set response */
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT");
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET, onoff %d", param->status_cb.onoff_status.present_onoff);
            int64_t receive_time = esp_timer_get_time(); /* Record the receive timestamp */
            int64_t rtt = receive_time - send_time;      /* Calculate round-trip time */
            ESP_LOGI(TAG, "Acknowledgment received at: %lld us", receive_time);
            ESP_LOGI(TAG, "Round-trip time: %lld us, Estimated one-way latency: %lld us", rtt, rtt / 2);
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
        /* Handle Generic Client publish event */
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT");
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        /* Handle timeout for Generic OnOff Set message */
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT");
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            /* Resend Generic OnOff Set message on timeout */
            example_ble_mesh_send_gen_onoff_set();
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Initialize the BLE Mesh stack and configure the provisioner
 */
static esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = { 0xdd, 0xdd }; /* Device UUID match pattern */
    esp_err_t err = ESP_OK;

    /* Initialize provisioning keys */
    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    /* Register BLE Mesh callbacks */
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(example_ble_mesh_generic_client_cb);

    /* Initialize the BLE Mesh stack */
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    /* Set the device UUID match pattern */
    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set matching device uuid");
        return err;
    }

    /* Enable the BLE Mesh provisioner */
    err = esp_ble_mesh_provisioner_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh provisioner");
        return err;
    }

    /* Add a local AppKey */
    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add local AppKey");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh on/off client initialized");

    return ESP_OK;
}

/**
 * @brief Main function for the BLE Mesh client
 */
void ble_mesh_client_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    /* Initialize NVS (Non-Volatile Storage) */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize Bluetooth */
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Get the device UUID */
    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the BLE Mesh stack */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}