set(srcs "main.c" "ble_mesh_client.c" "custom_mqtt_client.c" "board.c")

# Add the BLE Mesh component
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS ".")