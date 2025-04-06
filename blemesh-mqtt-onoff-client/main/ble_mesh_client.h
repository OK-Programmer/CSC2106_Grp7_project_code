#ifndef BLE_MESH_CLIENT_H
#define BLE_MESH_CLIENT_H

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"

void ble_mesh_client_main(void);
void example_ble_mesh_send_gen_onoff_set(void);

#endif // BLE_MESH_CLIENT_H