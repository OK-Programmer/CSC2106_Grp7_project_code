/* board.h - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C"
{
#endif /**< __cplusplus */

/* Include necessary drivers */
#include "driver/gpio.h"
#include "esp_ble_mesh_defs.h"

/* 
 * Board-specific GPIO pin mapping for LEDs
 * Each board model has different GPIO pins assigned for RGB LEDs
 */
#if defined(CONFIG_BLE_MESH_ESP_WROOM_32)
/* ESP-WROOM-32 development board LED pins */
#define LED_R GPIO_NUM_25  /* Red LED connected to GPIO 25 */
#define LED_G GPIO_NUM_26  /* Green LED connected to GPIO 26 */
#define LED_B GPIO_NUM_27  /* Blue LED connected to GPIO 27 */
#elif defined(CONFIG_BLE_MESH_ESP_WROVER)
/* ESP-WROVER development board LED pins */
#define LED_R GPIO_NUM_0   /* Red LED connected to GPIO 0 */
#define LED_G GPIO_NUM_2   /* Green LED connected to GPIO 2 */
#define LED_B GPIO_NUM_4   /* Blue LED connected to GPIO 4 */
#elif defined(CONFIG_BLE_MESH_ESP32C3_DEV)
/* ESP32-C3 development board LED pins */
#define LED_R GPIO_NUM_2   /* All LEDs connected to GPIO 2 on this board */
#define LED_G GPIO_NUM_2
#define LED_B GPIO_NUM_2
#elif defined(CONFIG_BLE_MESH_ESP32S3_DEV)
/* ESP32-S3 development board LED pins */
#define LED_R GPIO_NUM_47  /* All LEDs connected to GPIO 47 on this board */
#define LED_G GPIO_NUM_47
#define LED_B GPIO_NUM_47
#elif defined(CONFIG_BLE_MESH_ESP32C6_DEV)
/* ESP32-C6 development board LED pins */
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 on this board */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32C61_DEV)
/* ESP32-C61 development board LED pins */
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 on this board */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32H2_DEV)
/* ESP32-H2 development board LED pins */
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 on this board */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#elif defined(CONFIG_BLE_MESH_ESP32C5_DEV)
/* ESP32-C5 development board LED pins */
#define LED_R GPIO_NUM_8   /* All LEDs connected to GPIO 8 on this board */
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#endif

/* LED control states */
#define LED_ON  1          /* Value to turn LED on */
#define LED_OFF 0          /* Value to turn LED off */

    /**
     * Structure to track LED state
     * Keeps both current and previous state for state change detection
     */
    struct _led_state
    {
        uint8_t current;   /* Current state of the LED (ON/OFF) */
        uint8_t previous;  /* Previous state of the LED (ON/OFF) */
        uint8_t pin;       /* GPIO pin number for this LED */
        char *name;        /* Name identifier for this LED */
    };

    /**
     * Function to control LED state
     * @param pin: GPIO pin number of the LED
     * @param onoff: Desired state (LED_ON or LED_OFF)
     */
    void board_led_operation(uint8_t pin, uint8_t onoff);

    /**
     * Initialize board-specific configurations
     * Sets up GPIO pins, LED initial states, etc.
     */
    void board_init(void);

#ifdef __cplusplus
}
#endif /**< __cplusplus */

#endif /* _BOARD_H_ */
