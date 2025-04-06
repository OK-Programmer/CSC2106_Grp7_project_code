#include "ble_mesh_client.h"
#include "custom_mqtt_client.h"
#include "freertos/FreeRTOS.h" // Include FreeRTOS headers
#include "freertos/task.h"     // For vTaskDelay

void app_main(void)
{
    ble_mesh_client_main();
    mqtt_client_main();

    while (1)
    {
        if (get_is_provisioned())
        {
            example_ble_mesh_send_sensor_message(0);
            // vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
            vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay for 1 second
        }
        else
        {
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 100 milliseconds
        }
    }
}