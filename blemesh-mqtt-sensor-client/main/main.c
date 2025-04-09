/**
 * @file main.c
 * @brief Main application for BLE Mesh Sensor Client with MQTT connectivity
 */

#include "ble_mesh_client.h"    // BLE Mesh client functionality
#include "custom_mqtt_client.h"  // Custom MQTT client functionality
#include "freertos/FreeRTOS.h"   // FreeRTOS functionality
#include "freertos/task.h"       // FreeRTOS task management

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    // Initialize the BLE Mesh client
    ble_mesh_client_main();
    
    // Initialize the MQTT client
    mqtt_client_main();

    // Main application loop
    while (1)
    {
        // Check if the device is provisioned into a BLE Mesh network
        if (get_is_provisioned())
        {
            // Send sensor data request to mesh network
            example_ble_mesh_send_sensor_message(0);
            
            // Wait for 3 seconds before sending next request
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
        else
        {
            // Device not provisioned yet, check again after 5 seconds
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}