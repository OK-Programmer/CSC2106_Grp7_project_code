/**
 * @file main.c
 * @brief Main application entry point for BLE Mesh and MQTT client
 */

 #include "ble_mesh_client.h"  // Header for BLE mesh client functionality
 #include "custom_mqtt_client.h" // Header for custom MQTT client functionality
 
 /**
  * @brief Application main entry point
  * 
  * Initializes and starts both BLE mesh client and MQTT client
  */
 void app_main(void)
 {
     ble_mesh_client_main();  // Initialize and start BLE mesh client
     mqtt_client_main();      // Initialize and start MQTT client
 }