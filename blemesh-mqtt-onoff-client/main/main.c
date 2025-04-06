#include "ble_mesh_client.h"
#include "custom_mqtt_client.h"

void app_main(void)
{
    ble_mesh_client_main();
    mqtt_client_main();
}